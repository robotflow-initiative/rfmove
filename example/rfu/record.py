env.asset_channel.GetRFMoveColliders()
env._step()
colliders = env.asset_channel.data['colliders']
print(len(colliders))
for one in colliders:
    print(one['object_id'])
    for i in one['collider']:
        print(i['type'])
        if i['type'] == 'box':
            print(i['position'])
            print(i['rotation'])
            print(i['size'])
        elif i['type'] == 'sphere':
            print(i['position'])
            print(i['radius'])