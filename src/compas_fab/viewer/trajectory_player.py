from compas.colors import Color
from compas.datastructures import Mesh as CompasMesh
from compas.geometry import Cylinder
from compas.geometry import Frame
from compas.geometry import Polyline
from compas.geometry import Scale
from compas.geometry import Transformation
from compas.geometry import Translation
from compas_robots import Configuration
import copy

try:
    from compas_threejs.materials import LineMaterial
    from compas_threejs.materials import Material
    from compas_threejs.materials import PhysicalMaterial
    from compas_threejs.ui import Slider
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

    def __init__(self, robot_cell, trajectory=None, cell_state=None):
        if not HAS_THREEJS:
            raise ImportError("The 'compas_threejs' package is required to use the TrajectoryPlayer.")

        self.viewer = Viewer()
        self.robot_cell = robot_cell
        self.trajectory = trajectory
        self.cell_state = cell_state or robot_cell.default_cell_state
        
        self.link_id_map = {}
        self.pnp_data = {"workpieces": {}}
        
        # Extract the static scene upon initialization
        self._extract_robot()
        self._extract_tools()
        self._extract_rigid_bodies()

        if self.trajectory:
            self._setup_scrubber()

    def add_dynamic_workpieces(self, pnp_data, geometry_dict):
        """Injects pick-and-place dynamic objects into the viewer.
        
        Parameters
        ----------
        pnp_data : dict
            A dictionary defining the frame intervals and parents for the dynamic items.
        geometry_dict : dict
            A dictionary mapping workpiece names to their COMPAS mesh/geometry.
        """
        self.pnp_data = pnp_data
        
        for name, geometry in geometry_dict.items():
            self.viewer.add_geometry(geometry)
            self.link_id_map[name] = [{"geometry": geometry, "T_local": Transformation()}]
    
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
                        self.viewer.add_geometry(item)
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
                    self.viewer.add_geometry(item.copy(), material=PhysicalMaterial(color=Color(1.0, 1.0, 1.0), wireframe=True))
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

        # 1. AUTO-DEDUCE LINK NAME
        self.triad_track_link = track_link_name
        if not self.triad_track_link:
            self.triad_track_link = robot_cell.get_end_effector_link_name(group=group)

        # 2. AUTO-DEDUCE TCP OFFSET
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

        # 4. Add the Gradient Trace (If requested)
        if draw_trace and len(tcp_points) > 1:
            trace_line = Polyline(tcp_points)
            lines = trace_line.lines
            num_lines = len(lines)
            
            for i, line in enumerate(lines):
                if line.length < 0.0001:
                    continue 
                r, g, b = i / num_lines, 0.0, 1.0 - (i / num_lines)
                self.viewer.add_geometry(line, LineMaterial(color=Color(r, g, b), opacity=0.5))

        # 5. Add the Dynamic TCP Triad (If requested)
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

        # 1. Create the transparent material
        ghost_mat = Material(color=Color(*color), opacity=opacity)

        # 2. Get the very last point in the trajectory
        final_point = self.trajectory.points[-1]
        final_cfg = Configuration(
            joint_values=final_point.joint_values,
            joint_types=final_point.joint_types,
            joint_names=self.trajectory.joint_names
        )

        full_config = self.cell_state.robot_configuration.merged(final_cfg)
        model = self.robot_cell.robot_model

        # 3. Spawn Ghost Robot Links
        for link in model.iter_links():
            link_name = link.name
            if link_name in self.link_id_map and self.link_id_map[link_name]:
                link_frame = model.forward_kinematics(full_config, link_name=link_name)
                T_link = Transformation.from_frame(link_frame)
                
                for mesh_data in self.link_id_map[link_name]:
                    # Copy the base geometry so we don't accidentally move the real robot!
                    ghost_mesh = mesh_data['geometry'].copy()
                    ghost_mesh.transform(T_link * mesh_data['T_local'])
                    
                    # Pass material as a positional argument
                    self.viewer.add_geometry(ghost_mesh, ghost_mat)

        # 4. Spawn Ghost Tool Links
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

    # --------------------------------------------------------------------------
    # Scrubber Logic
    # --------------------------------------------------------------------------

    def _setup_scrubber(self):
        
        def scrub_callback(value):
            frame_index = int(value[0] if isinstance(value, list) else value)
            point = self.trajectory.points[frame_index]
            
            config = Configuration(
                joint_values=point.joint_values,
                joint_types=point.joint_types,
                joint_names=self.trajectory.joint_names
            )

            full_config = self.cell_state.robot_configuration.merged(config)
            model = self.robot_cell.robot_model

            # A. Update Robot Links
            for link in model.iter_links():
                link_name = link.name
                if link_name in self.link_id_map and self.link_id_map[link_name]:
                    link_frame = model.forward_kinematics(full_config, link_name=link_name)
                    T_link = Transformation.from_frame(link_frame)
                    for mesh_data in self.link_id_map[link_name]:
                        self.viewer.transform(mesh_data['geometry'], T_link * mesh_data['T_local'])
                        
            # B. Update Tool Links
            for tool_name, t_state in self.cell_state.tool_states.items():
                t_model = self.robot_cell.tool_models[tool_name]
                parent_link = t_model.connected_to
                
                parent_frame = model.forward_kinematics(config, link_name=parent_link)
                T_parent = Transformation.from_frame(parent_frame)
                T_attach = Transformation.from_frame(t_state.attachment_frame) if t_state.attachment_frame else Transformation()
                
                for t_link in t_model.iter_links():
                    unique_name = f"{tool_name}_{t_link.name}"
                    if unique_name in self.link_id_map:
                        for mesh_data in self.link_id_map[unique_name]:
                            T_final = T_parent * T_attach * mesh_data['T_local']
                            self.viewer.transform(mesh_data['geometry'], T_final)

            # C. Update Dynamic Workpieces
            if self.pnp_data and "workpieces" in self.pnp_data:
                for wp_name, data in self.pnp_data["workpieces"].items():
                    if wp_name not in self.link_id_map:
                        continue
                        
                    current_state = next((s for s in data["states"] if s["start_frame"] <= frame_index < s["end_frame"]), None)
                    if not current_state:
                        continue
                        
                    T_state = current_state["transform"]
                    
                    if current_state["parent"] == "world":
                        T_final = T_state
                    else:
                        tool_name = current_state["parent"]
                        t_model = self.robot_cell.tool_models[tool_name]
                        
                        flange_frame = model.forward_kinematics(full_config, link_name=t_model.connected_to)
                        T_flange = Transformation.from_frame(flange_frame)
                        
                        t_state = self.cell_state.tool_states[tool_name]
                        T_tool_attach = Transformation.from_frame(t_state.attachment_frame) if t_state.attachment_frame else Transformation()
                        
                        T_final = T_flange * T_tool_attach * T_state
                        
                    for mesh_data in self.link_id_map[wp_name]:
                        self.viewer.transform(mesh_data['geometry'], T_final * mesh_data['T_local'])
           
            # E. Update TCP Triad
            if hasattr(self, 'triad_objects') and self.triad_objects:
                flange_frame = model.forward_kinematics(full_config, link_name=self.triad_track_link)
                T_flange = Transformation.from_frame(flange_frame)
                T_tcp = T_flange * self.T_offset # <-- Just change this one line!
                
                for mesh in self.triad_objects:
                    self.viewer.transform(mesh, T_tcp)

        # Keep a reference to the callback so we can force-call it on initialization
        self._scrub_callback = scrub_callback
        
        slider = Slider(
            label="Playback", 
            min=0, max=len(self.trajectory.points) - 1, step=1,
            default_value=0, action=self._scrub_callback
        )
        self.viewer.add_ui_element(slider)