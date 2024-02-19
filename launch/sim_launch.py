from simple_launch import SimpleLauncher


def generate_launch_description():

    sl = SimpleLauncher(use_sim_time=True)

    sl.gz_launch(f"-r {sl.find('baxter_gz', 'baxter_world.sdf')}")

    return sl.launch_description()