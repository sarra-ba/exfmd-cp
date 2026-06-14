import carla, csv, time, math

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

rows = []
with open('/home/syfra/workspace/moad_workspace/uphf_trajectory.csv') as f:
    reader = csv.DictReader(f)
    for row in reader:
        rows.append(row)

# Filtrer et prendre 1 point sur 5
filtered = [rows[0]]
for row in rows[1:]:
    prev = filtered[-1]
    dx = float(row['carla_x']) - float(prev['carla_x'])
    dy = float(row['carla_y']) - float(prev['carla_y'])
    if math.sqrt(dx**2 + dy**2) > 1.0:
        filtered.append(row)

print(f'Points: {len(filtered)}')

bp = world.get_blueprint_library().find('vehicle.tesla.model3')
bp.set_attribute('color', '255,0,0')
start = filtered[0]
vehicle = world.spawn_actor(bp, carla.Transform(
    carla.Location(x=float(start['carla_x']), y=float(start['carla_y']), z=0.5),
    carla.Rotation(yaw=90.0 - float(start['heading']))))
print(f'Spawné id={vehicle.id}')

spectator = world.get_spectator()
prev = filtered[0]
try:
    for row in filtered:
        cx, cy = float(row['carla_x']), float(row['carla_y'])
        dx = cx - float(prev['carla_x'])
        dy = cy - float(prev['carla_y'])
        heading = math.degrees(math.atan2(dy, dx)) if abs(dx)+abs(dy) > 0.01 else 90 - float(row['heading'])
        vehicle.set_transform(carla.Transform(
            carla.Location(x=cx, y=cy, z=0.5),
            carla.Rotation(yaw=heading)))
        spectator.set_transform(carla.Transform(
            carla.Location(x=cx, y=cy, z=15),
            carla.Rotation(pitch=-90)))
        prev = row
        time.sleep(0.05)
except KeyboardInterrupt:
    pass
finally:
    vehicle.destroy()
    print('Done')
