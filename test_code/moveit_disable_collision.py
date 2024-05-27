def disable_collision_checking(scene, robot):
    acm = AllowedCollisionMatrix()
    link_names = robot.get_link_names()
    acm.entry_names = link_names
    acm.entry_values = []

    for _ in link_names:
        entry = AllowedCollisionEntry()
        entry.enabled = [True] * len(link_names)
        acm.entry_values.append(entry)

    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(planning_scene)
    print("Collision checking disabled")
    
def enable_collision_checking(scene, robot):
    acm = AllowedCollisionMatrix()
    link_names = robot.get_link_names()
    acm.entry_names = link_names
    acm.entry_values = []

    for _ in link_names:
        entry = AllowedCollisionEntry()
        entry.enabled = [False] * len(link_names)
        acm.entry_values.append(entry)

    planning_scene = PlanningScene()
    planning_scene.is_diff = True
    planning_scene.allowed_collision_matrix = acm
    scene.apply_planning_scene(planning_scene)
    print("Collision checking enabled")