import carla

client = carla.Client('carla_server', 2000)
client.set_timeout(2000.0)
world = client.get_world()
while True:
	world_snapshot = world.wait_for_tick()