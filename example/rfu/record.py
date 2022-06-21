#读取环境碰撞体
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

# 设置默认固体位置
env.game_object_channel.set_action(
    'SetTransform',
    id=78494578,
    scale=[0.1, 0.1, 0.1]
)
env.game_object_channel.set_action(
    'SetTransform',
    id=58447323,
    scale=[0.05, 0.05, 0.05]
)