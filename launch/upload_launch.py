from simple_launch import SimpleLauncher, GazeboBridge


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_gazebo_axes(x=0., y=0., z=.92, yaw=3.14)

    sl.declare_arg('sliders',False)

    with sl.group(ns='robot'):

        sl.robot_state_publisher('baxter_description', 'baxter.urdf.xacro', xacro_args={'gazebo': True})

        sl.spawn_gz_model('baxter', spawn_args = sl.gazebo_axes_args())
                
        bridges = [GazeboBridge.clock()]
        
        gz_js_topic = sl.name_join(GazeboBridge.model_prefix('baxter'),'/joint_state')
        bridges.append(GazeboBridge(gz_js_topic, '/robot/joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        for side in ('left','right'):
            bridges.append(GazeboBridge(f'{side}_arm/image', f'/cameras/{side}_hand_camera/image', 'sensor_msgs/Image', GazeboBridge.gz2ros))
        
        sl.create_gz_bridge(bridges, 'sensor_bridge')
                
        sl.node('baxter_gz', 'baxter_gz_bridge')
        
        with sl.group(if_arg='sliders'):
            for side in ('left', 'right'):
                sl.node('slider_publisher', name=side+'_pub', arguments=[sl.find('baxter_gz', side+'_cmd.yaml')])

    return sl.launch_description()