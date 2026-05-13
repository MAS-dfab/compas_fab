from compas.colors import Color
from compas.datastructures import Mesh as CompasMesh
from compas.geometry import Cylinder
from compas.geometry import Frame
from compas.geometry import Polyline
from compas.geometry import Scale
from compas.geometry import Transformation
from compas.geometry import Translation
from compas_robots import Configuration

try:
    from compas_threejs.materials import LineMaterial
    from compas_threejs.materials import Material
    from compas_threejs.materials import PhysicalMaterial
    from compas_threejs.ui import Slider
    from compas_threejs.ui import Timeline
    from compas_threejs.viewer import Viewer
    HAS_THREEJS = True
except ImportError:
    HAS_THREEJS = False


class TrajectoryPlayer:
    """A web-based visualizer for robot trajectories and dynamic scenes.
    
    Parameters
    ----------
    robot_cell : :class:`compas_fab.robots.RobotCell`
        The robot cell to visualize.
    trajectory : :class:`compas_fab.robots.JointTrajectory`, optional
        The trajectory to play back.
    cell_state : :class:`compas_fab.robots.RobotCellState`, optional
        The initial state of the robot cell. If not provided, the default state is used.
    """

    def __init__(self, robot_cell, trajectory=None, cell_state=None, use_cache=False, cache_fps=20):
        if not HAS_THREEJS:
            raise ImportError("The 'compas_threejs' package is required to use the TrajectoryPlayer.")

        self.viewer = Viewer()
        self.robot_cell = robot_cell
        self.trajectory = trajectory
        self.cell_state = cell_state or robot_cell.default_cell_state

        self.use_cache = use_cache
        self.cache_step = 1.0 / cache_fps if cache_fps > 0 else 0.05
        
        self.link_id_map = {}
        self.pnp_data = {"workpieces": {}}
        
        self._extract_robot()
        self._extract_tools()
        self._extract_rigid_bodies()

    def add_dynamic_workpieces(self, pnp_data, geometry_dict):
        """Adds dynamic workpieces that attach and detach from the robot."""

        self.dynamic_workpieces = {}
        
        T_shadow_realm = Translation.from_vector([0, 0, -100])

        for item_name, mesh in geometry_dict.items():
            if item_name in pnp_data:
                wood_mat = PhysicalMaterial(color=Color(0.8, 0.6, 0.4), roughness=0.8, thickness=1.0)
                self.viewer.add_geometry(mesh, wood_mat) 
                
                self.dynamic_workpieces[item_name] = {
                    'mesh': mesh,
                    'rules': pnp_data[item_name]
                }
                print(f"🪵 Added dynamic workpiece: {item_name}")
                
                if item_name in self.link_id_map:
                    for mesh_data in self.link_id_map[item_name]:
                        # Permanently move the static grey mesh underground 
                        self.viewer.transform(mesh_data['geometry'], T_shadow_realm)
                        
            else:
                print(f"⚠️ {item_name} found in geometry_dict but not in pnp_data!")
    
    def add_visual_helpers(self, trace=True, triad=True, ghost=True, group=None, track_link_name=None, tcp_offset=None):
        """A convenient wrapper to add multiple visual aids to the scene at once.
        
        Parameters
        ----------
        trace : bool
            If True, draws a gradient polyline along the TCP path.
        triad : bool
            If True, attaches an RGB axis triad to the TCP.
        ghost : bool
            If True, draws a translucent robot at the final trajectory state.
        group : str, optional
            The planning group name, used to auto-deduce the correct tool link.
        track_link_name : str, optional
            Manual override for the link to track.
        tcp_offset : list or :class:`compas.geometry.Frame`, optional
            Manual override for the TCP offset.
        """
        if trace or triad:
            self.add_tcp_features(
                local_offset=tcp_offset,
                track_link_name=track_link_name,
                group=group,
                draw_trace=trace,
                draw_triad=triad
            )
            
        if ghost:
            self.add_target_ghost()

    def show(self):
        """Starts the local web server and opens the viewer in the browser."""
        
        if self.trajectory:
            self._setup_scrubber()

        # Force the scrubber to frame 0 so it initializes in the correct position
        if self.trajectory and hasattr(self, '_scrub_callback'):
            self._scrub_callback([0])
            
        self.viewer.start(show=False)

    # --------------------------------------------------------------------------
    # Extraction Helpers
    # --------------------------------------------------------------------------

    def _add_model_to_viewer(self, model_to_parse, name_prefix=""):
        """Helper to extract visuals from a kinematic model and load them into Three.js."""
        for link in model_to_parse.iter_links():
            unique_name = f"{name_prefix}{link.name}"
            
            if "tool" in unique_name:
                pmaterial = PhysicalMaterial(color=Color(0.4, 0.4, 0.4))
            elif "link" in unique_name:
                pmaterial = PhysicalMaterial(color=Color(0.6, 0.6, 0.6))
            elif "world" in unique_name:
                pmaterial = PhysicalMaterial(color=Color(0.8, 0.8, 0.8), opacity=0.2)
            else:
                pmaterial = PhysicalMaterial(color=Color(0.9, 0.9, 0.9), opacity=0.9)
            self.link_id_map[unique_name] = []
            
            for visual in link.visual:
                shape = visual.geometry.shape
                
                T_scale = Scale.from_factors(shape.scale) if hasattr(shape, 'scale') and shape.scale else Transformation()
                T_origin = Transformation()
                
                frame = getattr(visual, 'init_frame', None)
                if not frame:
                    origin = getattr(visual, 'origin', None)
                    if origin:
                        frame = getattr(origin, 'frame', origin)
                        
                if frame:
                    try:
                        T_origin = Transformation.from_frame(frame)
                    except Exception:
                        pass
                
                T_local = T_origin * T_scale
                meshes_to_add = shape.meshes if hasattr(shape, 'meshes') else [shape]
                
                for item in meshes_to_add:
                    if item is not None:
                        self.viewer.add_geometry(item, material=pmaterial)
                        self.link_id_map[unique_name].append({"geometry": item, "T_local": T_local})

    def _extract_robot(self):
        self._add_model_to_viewer(self.robot_cell.robot_model)

    def _extract_tools(self):
        for tool_id, tool_mod in self.robot_cell.tool_models.items():
            self._add_model_to_viewer(tool_mod, name_prefix=f"{tool_id}_")

    def _extract_rigid_bodies(self):
        for rb_name, rb_model in self.robot_cell.rigid_body_models.items():
            self.link_id_map[rb_name] = []
            rb_state = self.cell_state.rigid_body_states[rb_name]
            meshes_to_add = []
            
            if hasattr(rb_model, 'visual_meshes') and rb_model.visual_meshes:
                for wrapped_item in rb_model.visual_meshes:
                    if hasattr(wrapped_item, 'mesh'): meshes_to_add.append(wrapped_item.mesh)
                    elif hasattr(wrapped_item, 'geometry'): meshes_to_add.append(wrapped_item.geometry)
                    else: meshes_to_add.append(wrapped_item)
            elif hasattr(rb_model, 'mesh') and rb_model.mesh:
                meshes_to_add.append(rb_model.mesh)
                            
            seen_guids = set()
            for item in meshes_to_add:
                if item is not None and hasattr(item, 'guid'):
                    guid_str = str(item.guid)
                    if guid_str in seen_guids: continue
                    seen_guids.add(guid_str)
                    
                    self.viewer.add_geometry(item, material=PhysicalMaterial(color=Color(0.5, 0.5, 0.5)))
                    self.link_id_map[rb_name].append({"geometry": item, "T_local": Transformation()})
                    
                    if rb_state.frame:
                        T_world = Transformation.from_frame(rb_state.frame)
                        self.viewer.transform(item, T_world)

    # --------------------------------------------------------------------------
    # Visualisation Helpers
    # --------------------------------------------------------------------------

    def add_tcp_features(self, local_offset=None, track_link_name=None, group=None, draw_trace=True, draw_triad=True):
        """Adds a static path trace and a dynamic TCP triad to the viewer."""
        if not self.trajectory or not self.trajectory.points:
            return

        robot_cell = self.robot_cell
        model = self.robot_cell.robot_model

        # 1. LINK NAME
        self.triad_track_link = track_link_name
        if not self.triad_track_link:
            self.triad_track_link = robot_cell.get_end_effector_link_name(group=group)

        # 2. TCP OFFSET
        if local_offset is not None:
            if isinstance(local_offset, Frame):
                self.T_offset = Transformation.from_frame(local_offset)
            else:
                self.T_offset = Translation.from_vector(local_offset)
        else:
            active_tool = next((t for t in self.robot_cell.tool_models.values() if t.connected_to == self.triad_track_link), None)
            if active_tool and active_tool.frame:
                self.T_offset = Transformation.from_frame(active_tool.frame)
            else:
                self.T_offset = Transformation()

        # 3. Calculate TCP Points
        tcp_points = []
        for point in self.trajectory.points:
            cfg = Configuration(
                joint_values=point.joint_values,
                joint_types=point.joint_types,
                joint_names=self.trajectory.joint_names
            )
            full_config = self.cell_state.robot_configuration.merged(cfg)
            flange_frame = model.forward_kinematics(full_config, link_name=self.triad_track_link)
            
            T_flange = Transformation.from_frame(flange_frame)
            T_tcp = T_flange * self.T_offset
            tcp_points.append(Frame.from_transformation(T_tcp).point)

        # 4. Add the Gradient Trace
        if draw_trace and len(tcp_points) > 1:
            self.trace_objects = getattr(self, 'trace_objects', [])
            trace_line = Polyline(tcp_points)
            lines = trace_line.lines
            num_lines = len(lines)
            
            for i, line in enumerate(lines):
                length = line.length
                if length < 0.0001:
                    continue 

                # 1. Create a thin cylinder (2mm radius) centered at the origin
                cyl_shape = Cylinder(0.002, length) 
                trace_mesh = CompasMesh.from_shape(cyl_shape)

                # 2. Mathematically orient the cylinder to perfectly match the line segment
                z_axis = line.vector.unitized()
                if abs(z_axis.z) < 0.99:
                    x_axis = z_axis.cross([0, 0, 1]).unitized()
                else:
                    x_axis = z_axis.cross([0, 1, 0]).unitized()
                y_axis = z_axis.cross(x_axis).unitized()

                target_frame = Frame(line.midpoint, x_axis, y_axis)
                trace_mesh.transform(Transformation.from_frame(target_frame))

                # 3. Apply your beautiful gradient and send it to the viewer
                r, g, b = i / num_lines, 0.0, 1.0 - (i / num_lines)
                mat = PhysicalMaterial(color=Color(r, g, b))
                
                self.viewer.add_geometry(trace_mesh, mat)
                self.trace_objects.append(trace_mesh)

        # 5. Add the Dynamic TCP Triad
        if draw_triad:
            base_cyl = Cylinder(0.005, 0.2)
            
            x_mesh = CompasMesh.from_shape(base_cyl)
            x_mesh.transform(Transformation.from_frame(Frame([0,0,0], [0,1,0], [0,0,1])) * Translation.from_vector([0, 0, 0.1]))
            
            y_mesh = CompasMesh.from_shape(base_cyl)
            y_mesh.transform(Transformation.from_frame(Frame([0,0,0], [0,0,1], [1,0,0])) * Translation.from_vector([0, 0, 0.1]))
            
            z_mesh = CompasMesh.from_shape(base_cyl)
            z_mesh.transform(Translation.from_vector([0, 0, 0.1]))
            
            self.viewer.add_geometry(x_mesh, PhysicalMaterial(color=Color(1.0, 0.0, 0.0)))
            self.viewer.add_geometry(y_mesh, PhysicalMaterial(color=Color(0.0, 1.0, 0.0)))
            self.viewer.add_geometry(z_mesh, PhysicalMaterial(color=Color(0.0, 0.0, 1.0)))
            
            self.triad_objects = [x_mesh, y_mesh, z_mesh]

    def add_target_ghost(self, color=(0.5, 0.7, 0.9), opacity=0.2):
        """Adds a translucent 'ghost' of the robot and tools at the final trajectory state."""
        if not self.trajectory or not self.trajectory.points:
            print("⚠️ No trajectory points found for ghost!")
            return
        
        self.ghost_objects = getattr(self, 'ghost_objects', [])
        ghost_mat = Material(color=Color(*color), opacity=opacity)

        final_point = self.trajectory.points[-1]
        final_cfg = Configuration(
            joint_values=final_point.joint_values,
            joint_types=final_point.joint_types,
            joint_names=self.trajectory.joint_names
        )

        full_config = self.cell_state.robot_configuration.merged(final_cfg)
        model = self.robot_cell.robot_model

        for link in model.iter_links():
            link_name = link.name
            if link_name in self.link_id_map and self.link_id_map[link_name]:
                link_frame = model.forward_kinematics(full_config, link_name=link_name)
                T_link = Transformation.from_frame(link_frame)
                
                for mesh_data in self.link_id_map[link_name]:
                    ghost_mesh = mesh_data['geometry'].copy()
                    ghost_mesh.transform(T_link * mesh_data['T_local'])
                    self.viewer.add_geometry(ghost_mesh, ghost_mat)

        for tool_name, t_state in self.cell_state.tool_states.items():
            t_model = self.robot_cell.tool_models[tool_name]
            parent_link = t_model.connected_to
            
            parent_frame = model.forward_kinematics(full_config, link_name=parent_link)
            T_parent = Transformation.from_frame(parent_frame)
            T_attach = Transformation.from_frame(t_state.attachment_frame) if t_state.attachment_frame else Transformation()
            
            for t_link in t_model.iter_links():
                unique_name = f"{tool_name}_{t_link.name}"
                if unique_name in self.link_id_map:
                    for mesh_data in self.link_id_map[unique_name]:
                        ghost_tool_mesh = mesh_data['geometry'].copy()
                        T_final = T_parent * T_attach * mesh_data['T_local']
                        ghost_tool_mesh.transform(T_final)
                        
                        self.viewer.add_geometry(ghost_tool_mesh, ghost_mat)
                        self.ghost_objects.append(ghost_mesh)

    def _cleanup_previous_run(self):
        """Completely removes old geometries from the Three.js scene."""

        # 1. REMOVE old assembled elements
        if hasattr(self, 'assembled_objects'):
            for mesh in self.assembled_objects:
                self._hide_and_remove(mesh)
            self.assembled_objects = []
                
        # 2. REMOVE and CLEAR old workpieces
        if hasattr(self, 'dynamic_workpieces'):
            for data in self.dynamic_workpieces.values():
                self._hide_and_remove(data['mesh'])
            self.dynamic_workpieces = {} 
                
        # 3. REMOVE old TCP triad
        if hasattr(self, 'triad_objects'):
            for mesh in self.triad_objects:
                self._hide_and_remove(mesh)
            self.triad_objects = []

        # 4. REMOVE old traces
        if hasattr(self, 'trace_objects'):
            for line in self.trace_objects:
                self._hide_and_remove(line)
            self.trace_objects = []

        # 5. REMOVE old ghosts
        if hasattr(self, 'ghost_objects'):
            for mesh in self.ghost_objects:
                self._hide_and_remove(mesh)
            self.ghost_objects = []

    def _draw_assembled_elements(self, meshes):
        """Adds the newly calculated assembled elements to the frontend viewer."""
        self.assembled_objects = getattr(self, 'assembled_objects', [])
        
        wood_mat = PhysicalMaterial(color=Color(0.5, 0.5, 0.5)) # Gray for already assembled
        
        for mesh in meshes:
            self.viewer.add_geometry(mesh, material=wood_mat)
            self.assembled_objects.append(mesh)
    
    def _hide_and_remove(self, obj):
        """A bulletproof method to remove objects from the Three.js canvas."""
        if not obj: return
        
        # 1. GOLD STANDARD: Use your new direct websocket dispatch via GUID
        try:
            self.viewer.remove_object(obj)
        except Exception:
            pass

        # 2. FALLBACK: Teleport to the shadow realm
        try:
            self.viewer.transform(obj, Translation.from_vector([0, 0, -10000]))
            if hasattr(obj, 'visible'):
                obj.visible = False
        except Exception: 
            pass

    # --------------------------------------------------------------------------
    # Scrubber Logic
    # --------------------------------------------------------------------------
    
    def _get_time_in_seconds(self, point):
        """Safely extracts the absolute time in seconds from a trajectory point."""
        duration = point.time_from_start
        if duration is None:
            return 0.0
        return duration.seconds

    def _get_interpolated_config(self, t):
        """Generates an exact robot Configuration for any given time 't'."""
        points = self.trajectory.points

        if t <= 0.0:
            return Configuration(points[0].joint_values, points[0].joint_types, self.trajectory.joint_names)
        
        total_time = self._get_time_in_seconds(points[-1])
        if t >= total_time:
            return Configuration(points[-1].joint_values, points[-1].joint_types, self.trajectory.joint_names)

        for i in range(len(points) - 1):
            t0 = self._get_time_in_seconds(points[i])
            t1 = self._get_time_in_seconds(points[i+1])
            
            if t0 <= t <= t1:
                dt = t1 - t0
                ratio = (t - t0) / dt if dt > 0 else 0.0
                
                vals0 = points[i].joint_values
                vals1 = points[i+1].joint_values
                
                interp_vals = [v0 + ratio * (v1 - v0) for v0, v1 in zip(vals0, vals1)]
                
                return Configuration(interp_vals, points[i].joint_types, self.trajectory.joint_names)
        
        return Configuration(points[-1].joint_values, points[-1].joint_types, self.trajectory.joint_names)
    
    def _calculate_frame_transforms(self, t):
        """Pure math engine. Returns a list of (mesh, Transformation) tuples for a given time."""
        frame_transforms = []
        
        interp_cfg = self._get_interpolated_config(t)
        full_config = self.cell_state.robot_configuration.merged(interp_cfg)
        model = self.robot_cell.robot_model

        # 1. Update the Robot
        for link in model.iter_links():
            link_name = link.name
            if link_name in self.link_id_map and self.link_id_map[link_name]:
                link_frame = model.forward_kinematics(full_config, link_name=link_name)
                T_link = Transformation.from_frame(link_frame)
                
                for mesh_data in self.link_id_map[link_name]:
                    frame_transforms.append((mesh_data['geometry'], T_link * mesh_data['T_local']))

        # 2. Update the Tools
        for tool_name, t_state in self.cell_state.tool_states.items():
            t_model = self.robot_cell.tool_models[tool_name]
            parent_link = t_model.connected_to
            
            parent_frame = model.forward_kinematics(full_config, link_name=parent_link)
            T_parent = Transformation.from_frame(parent_frame)
            T_attach = Transformation.from_frame(t_state.attachment_frame) if t_state.attachment_frame else Transformation()
            
            for t_link in t_model.iter_links():
                unique_name = f"{tool_name}_{t_link.name}"
                if unique_name in self.link_id_map:
                    t_link_frame = t_model.forward_kinematics(full_config, link_name=t_link.name)
                    T_t_link = Transformation.from_frame(t_link_frame)
                    T_final = T_parent * T_attach * T_t_link
                    
                    for mesh_data in self.link_id_map[unique_name]:
                        frame_transforms.append((mesh_data['geometry'], T_final * mesh_data['T_local']))

        # 3. Update TCP Triad
        if hasattr(self, 'triad_objects') and self.triad_objects and hasattr(self, 'triad_track_link'):
            flange_frame = model.forward_kinematics(full_config, link_name=self.triad_track_link)
            T_flange = Transformation.from_frame(flange_frame)
            T_tcp = T_flange * getattr(self, 'T_offset', Transformation())
            
            for mesh in self.triad_objects:
                frame_transforms.append((mesh, T_tcp))

        # 4. Update Dynamic Workpieces
        if hasattr(self, 'dynamic_workpieces') and self.dynamic_workpieces:
            track_link = getattr(self, 'triad_track_link', model.get_end_effector_link_name())
            
            active_tool = next((t for t in self.robot_cell.tool_models.values() if t.connected_to == track_link), None)
            T_tool_offset = Transformation.from_frame(active_tool.frame) if active_tool and active_tool.frame else Transformation()
            
            flange_frame = model.forward_kinematics(full_config, link_name=track_link)
            T_flange = Transformation.from_frame(flange_frame)

            for name, data in self.dynamic_workpieces.items():
                rules = data['rules']
                attach_time = rules.get('attach_time', -1.0)
                detach_time = rules.get('detach_time', float('inf'))
                T_grasp = rules.get('T_grasp', Transformation())
                
                T_park = rules.get('T_park', None) 
                appear_time = rules.get('appear_time', 0.0)
                vanish_delay = rules.get('vanish_delay', None)
                
                from compas.geometry import Translation
                T_shadow_realm = Translation.from_vector([0, 0, -100])
                
                if t < appear_time:
                    if T_park is not None:
                        T_object = T_park
                    else:
                        T_object = T_shadow_realm
                        
                elif t < attach_time:
                    cfg_pickup = self._get_interpolated_config(attach_time)
                    full_cfg_pickup = self.cell_state.robot_configuration.merged(cfg_pickup)
                    f_pickup = model.forward_kinematics(full_cfg_pickup, link_name=track_link)
                    T_object = Transformation.from_frame(f_pickup) * T_tool_offset * T_grasp
                    
                elif vanish_delay is not None and t > (detach_time + vanish_delay):
                    T_object = T_shadow_realm
                    
                elif t >= detach_time and detach_time != float('inf'):
                    cfg_drop = self._get_interpolated_config(detach_time)
                    full_cfg_drop = self.cell_state.robot_configuration.merged(cfg_drop)
                    f_drop = model.forward_kinematics(full_cfg_drop, link_name=track_link)
                    T_object = Transformation.from_frame(f_drop) * T_tool_offset * T_grasp
                    
                else:
                    T_object = T_flange * T_tool_offset * T_grasp
                    
                frame_transforms.append((data['mesh'], T_object))

        return frame_transforms
    
    def _setup_scrubber(self):
        if not self.trajectory or not self.trajectory.points:
            return

        total_time = self._get_time_in_seconds(self.trajectory.points[-1])

        # ==========================================
        # PATH A: CACHED PLAYBACK
        # ==========================================
        if getattr(self, 'use_cache', False):
            step_size = self.cache_step
            print(f"🧠 Pre-calculating {int(total_time / step_size)} animation frames... please wait...")
            
            self.frame_cache = {}
            current_t = 0.0
            
            while current_t <= total_time + step_size:
                t = min(current_t, total_time)
                self.frame_cache[round(t, 2)] = self._calculate_frame_transforms(t)
                
                if t == total_time: 
                    break
                current_t += step_size
                
            print("✅ Cache built! Ready for instant playback.")

            def scrub_callback(t_value):
                t_req = t_value[0] if isinstance(t_value, list) else t_value
                t_rounded = round(round(t_req / step_size) * step_size, 2)
                
                if t_rounded > round(total_time, 2):
                    t_rounded = round(round(total_time / step_size) * step_size, 2)
                    
                cached_transforms = self.frame_cache.get(t_rounded)
                if cached_transforms:
                    for mesh, T in cached_transforms:
                        self.viewer.transform(mesh, T)

        # ==========================================
        # PATH B: LIVE PLAYBACK
        # ==========================================
        else:
            def scrub_callback(t_value):
                t = t_value[0] if isinstance(t_value, list) else t_value
                
                transforms = self._calculate_frame_transforms(t)
                for mesh, T in transforms:
                    self.viewer.transform(mesh, T)

        mode_str = "Cached" if getattr(self, 'use_cache', False) else "Live"
        print(f"⏱️ Creating time-based scrubber (Total Time: {total_time:.2f}s, Mode: {mode_str})")
        
        timeline = Timeline(total_time=total_time, step=0.01, value=0.0, action=scrub_callback)
        self.viewer.add_ui_element(timeline)
        
        scrub_callback(0.0)

    # def _setup_scrubber(self):
    #     if not self.trajectory or not self.trajectory.points:
    #         return

    #     total_time = self._get_time_in_seconds(self.trajectory.points[-1])
    #     model = self.robot_cell.robot_model

    #     def scrub_callback(t_value):
    #         t = t_value[0] if isinstance(t_value, list) else t_value

    #         interp_cfg = self._get_interpolated_config(t)
    #         full_config = self.cell_state.robot_configuration.merged(interp_cfg)

    #         # 1. Update the Robot
    #         for link in model.iter_links():
    #             link_name = link.name
    #             if link_name in self.link_id_map and self.link_id_map[link_name]:
    #                 link_frame = model.forward_kinematics(full_config, link_name=link_name)
    #                 T_link = Transformation.from_frame(link_frame)
                    
    #                 for mesh_data in self.link_id_map[link_name]:
    #                     self.viewer.transform(mesh_data['geometry'], T_link * mesh_data['T_local'])

    #         # 2. Update the Tools (This brings your gripper back!)
    #         for tool_name, t_state in self.cell_state.tool_states.items():
    #             t_model = self.robot_cell.tool_models[tool_name]
    #             parent_link = t_model.connected_to
                
    #             parent_frame = model.forward_kinematics(full_config, link_name=parent_link)
    #             T_parent = Transformation.from_frame(parent_frame)
    #             T_attach = Transformation.from_frame(t_state.attachment_frame) if t_state.attachment_frame else Transformation()
                
    #             for t_link in t_model.iter_links():
    #                 unique_name = f"{tool_name}_{t_link.name}"
    #                 if unique_name in self.link_id_map:
    #                     t_link_frame = t_model.forward_kinematics(full_config, link_name=t_link.name)
    #                     T_t_link = Transformation.from_frame(t_link_frame)
    #                     T_final = T_parent * T_attach * T_t_link
                        
    #                     for mesh_data in self.link_id_map[unique_name]:
    #                         self.viewer.transform(mesh_data['geometry'], T_final * mesh_data['T_local'])

    #         # 3. Update TCP Triad
    #         if hasattr(self, 'triad_objects') and self.triad_objects and hasattr(self, 'triad_track_link'):
    #             flange_frame = model.forward_kinematics(full_config, link_name=self.triad_track_link)
    #             T_flange = Transformation.from_frame(flange_frame)
    #             T_tcp = T_flange * getattr(self, 'T_offset', Transformation())
                
    #             for mesh in self.triad_objects:
    #                 self.viewer.transform(mesh, T_tcp)

    #         # 4. Update Dynamic Workpieces
    #         if hasattr(self, 'dynamic_workpieces') and self.dynamic_workpieces:
    #             track_link = getattr(self, 'triad_track_link', model.get_end_effector_link_name())
                
    #             # Robustly find the Tool offset independent of the Triad!
    #             active_tool = next((t for t in self.robot_cell.tool_models.values() if t.connected_to == track_link), None)
    #             T_tool_offset = Transformation.from_frame(active_tool.frame) if active_tool and active_tool.frame else Transformation()
                
    #             flange_frame = model.forward_kinematics(full_config, link_name=track_link)
    #             T_flange = Transformation.from_frame(flange_frame)

    #             for name, data in self.dynamic_workpieces.items():
    #                 rules = data['rules']
    #                 attach_time = rules.get('attach_time', -1.0)
    #                 detach_time = rules.get('detach_time', float('inf'))
    #                 T_grasp = rules.get('T_grasp', Transformation())
                    
    #                 # 1. Look for a Lumber Yard spot. Default to None!
    #                 T_park = rules.get('T_park', None) 
    #                 appear_time = rules.get('appear_time', 0.0)
    #                 vanish_delay = rules.get('vanish_delay', None)
                    
    #                 from compas.geometry import Translation
    #                 T_shadow_realm = Translation.from_vector([0, 0, -100])
                    
    #                 if t < appear_time:
    #                     if T_park is not None:
    #                         # 1. STOCK: Wait in the Lumber Yard until 3 seconds before pickup!
    #                         T_object = T_park
    #                     else:
    #                         # 2. ELEMENTS: Hide underground until the Stock gets milled!
    #                         T_object = T_shadow_realm
                            
    #                 elif t < attach_time:
    #                     # 3. DELIVERY TO CNC BED! 
    #                     # (Stock arrives 3s early. Elements appear exactly when milled).
    #                     cfg_pickup = self._get_interpolated_config(attach_time)
    #                     full_cfg_pickup = self.cell_state.robot_configuration.merged(cfg_pickup)
    #                     f_pickup = model.forward_kinematics(full_cfg_pickup, link_name=track_link)
    #                     T_object = Transformation.from_frame(f_pickup) * T_tool_offset * T_grasp
                        
    #                 elif vanish_delay is not None and t > (detach_time + vanish_delay):
    #                     # 4. MILLED AWAY: The stock vanishes into the Shadow Realm!
    #                     T_object = T_shadow_realm
                        
    #                 elif t >= detach_time and detach_time != float('inf'):
    #                     # 5. DROPPED OFF: Sitting on the machine or Assembly Table
    #                     cfg_drop = self._get_interpolated_config(detach_time)
    #                     full_cfg_drop = self.cell_state.robot_configuration.merged(cfg_drop)
    #                     f_drop = model.forward_kinematics(full_cfg_drop, link_name=track_link)
    #                     T_object = Transformation.from_frame(f_drop) * T_tool_offset * T_grasp
                        
    #                 else:
    #                     # 6. IN TRANSIT: Attached to the gripper TCP!
    #                     T_object = T_flange * T_tool_offset * T_grasp
                        
    #                 # Apply the calculated transformation to the Three.js mesh
    #                 self.viewer.transform(data['mesh'], T_object)

    #     print(f"⏱️ Creating time-based scrubber (Total Time: {total_time:.2f}s)")
    #     # slider = Slider(title="Time (s)", min=0.0, max=total_time, step=0.01, value=0.0, action=scrub_callback)
    #     timeline = Timeline(total_time=total_time, step=0.01, value=0.0, action=scrub_callback)
        
    #     self.viewer.add_ui_element(timeline)
    #     scrub_callback(0.0)