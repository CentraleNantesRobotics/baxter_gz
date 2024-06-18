from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.gz_launch(f"-r {sl.find('baxter_gz', 'baxter_world.sdf')}")

    sl.include('baxter_gz', 'upload_launch.py')

    return sl.launch_description()
