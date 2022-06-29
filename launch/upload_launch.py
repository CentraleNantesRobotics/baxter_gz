import os
from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time=True)

    sl.declare_arg('x', 0.)
    sl.declare_arg('y', 0.)
    sl.declare_arg('z', 0.93)
    sl.declare_arg('yaw', 0.)
    
    sl.declare_arg('sliders',False)
            
    with sl.group(ns='robot'):
        
        sl.include('baxter_description', 'baxter_state_publisher_launch.py')                

        sl.spawn_gz_model('baxter',
                          spawn_args = [sl.name_join(f'-{tag} ', sl.arg(axis)) for axis,tag in (('x','x'),('y','y'),('z','z'),('yaw','Y'))])
        
        
        bridges = []
        
        gz_js_topic = sl.name_join(GazeboBridge.model_prefix('robot'),'/joint_state')
        bridges.append(GazeboBridge(gz_js_topic, '/robot/joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        # TODO run image / sensor bridges        
        
        sl.create_gz_bridge(bridges)
                
        sl.node('baxter_gazebosim', 'baxter_gz_bridge')
        
        with sl.group(if_arg='sliders'):
            for side in ('left', 'right'):
                sl.node('slider_publisher', name=side+'_pub', arguments=[sl.find('baxter_gazebosim', side+'_cmd.yaml')])

    return sl.launch_description()
