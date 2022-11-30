import numpy as np
from boid import Boid_visual as Boid
# import PySimpleGUIWeb as sg
import PySimpleGUI as sg

width = 1000
height = 1000
starting_num_birds = 40

size_choices = {'1000x1000':1000, '800x800':800, '400x400':400, '200x200':200}

def draw(window, flock):
    flock.edges()
    flock.update()
    flock.show(window)


def main(count):
    layout = [[sg.Text('What size of window would you like?')],
                [sg.Combo(list(size_choices.keys()), key='_COMBO_')],
                [sg.OK(), sg.Cancel()]]
    window = sg.Window('Choose size of window', layout)
    event, values = window.Read()
    window.Close()
    if event in (None, 'Cancel'):
        exit()
    width = height = size_choices[values['_COMBO_']]

    current_num_birds = count

    layout =    [[sg.Text('Boid Flocking'), sg.Text('Number Birds = '), sg.Text('', size=(4,1), key='_NUM_BIRDS_')],
                [sg.Graph((width,height), (0,0), (width,height), background_color='GhostWhite', key='_GRAPH_')],
                [sg.Text('iteration', key='_ITER_'),
                sg.T('Polarization', key='_POLAR_')],
                [sg.T('Max Speed (4)'), sg.Slider(range=(1,30), default_value=4, orientation='h', key='_SLIDER_SPEED_', enable_events=True),
                sg.T('Min Speed (3)'), sg.Slider(range=(1,30),default_value=3,  orientation='h', key='_SLIDER_MIN_', enable_events=True)],
                [sg.T('Number of birds'), sg.Slider(range=(4, 80), orientation='h', default_value=starting_num_birds, key='_SLIDER_', enable_events=True),
                sg.T('Centering Strength (0.0005)'), sg.Slider(range=(0, 1.0005), default_value=0.0005, resolution=.0005, orientation='h', key='_SLIDER_CENTER_', enable_events=True)],
                [sg.T('Velocity Match (0.05)'), sg.Slider(range=(0, 10), default_value=0.05, resolution=0.05, orientation='h', key='_SLIDER_VELOCITY_', enable_events=True),
                sg.T('Avoid Strength (0.05)'), sg.Slider(range=(0, 10), default_value=0.05, resolution=.05, orientation='h', key='_SLIDER_AVOID_', enable_events=True)],
                [sg.T('Turn Factor (0.05)'), sg.Slider(range=(0, 1), default_value=0.1, resolution=.001, orientation='h', key='_SLIDER_TURN_', enable_events=True),
                sg.T('Perception Distance (75)'), sg.Slider(range=(0, 200), default_value=100, resolution=15, orientation='h', key='_SLIDER_PERCEPTION_', enable_events=True)],
                 [sg.Exit()],]

    window = sg.Window('Boids', layout)
    window.Finalize()
    # graph = window.Element('_GRAPH_')               # type: sg.Graph
    flock = Boid(count, window, width, height)   # type: Boid:list

    # initalize to match the GUI values
    event, values = window.Read(timeout=0)
    min_speed = float(values['_SLIDER_MIN_'])
    max_speed = float(values['_SLIDER_SPEED_'])
    perception = float(values['_SLIDER_PERCEPTION_'])
    centering = float(values['_SLIDER_CENTER_'])
    velocity = float(values['_SLIDER_VELOCITY_'])
    avoid = float(values['_SLIDER_AVOID_'])
    turn = float(values['_SLIDER_TURN_'])

    flock.max_speed = max_speed
    flock.min_speed = min_speed
    flock.turnfactor = turn
    flock.perception = perception
    flock.move_to_middle_strength = centering  # for towards center of mass
    flock.formation_flying_strength = velocity  # for velocity matching
    flock.avoid_strength = avoid

    while True:
        event, values = window.Read(timeout=0)
        window.Element('_ITER_').update(value=flock.iterations)
        window.Element('_POLAR_').update(value=flock.polarization())
        if event in (None, 'Exit'):
            break
        # if event == '_SLIDER_':
        #     num_birds = int(values['_SLIDER_'])
        #     if num_birds > current_num_birds:
        #         flock = flock + [Boid(*np.random.rand(2)*1000, width, height) for _ in range(num_birds-current_num_birds)]
        #     else:
        #         for i in range(current_num_birds-num_birds):
        #             graph.DeleteFigure(flock[-1].drawing_id)
        #             del flock[-1]
        #     current_num_birds = num_birds
        elif event.startswith('_SLIDER_'):
            min_speed = float(values['_SLIDER_MIN_'])
            max_speed = float(values['_SLIDER_SPEED_'])
            perception = float(values['_SLIDER_PERCEPTION_'])
            centering = float(values['_SLIDER_CENTER_'])
            velocity = float(values['_SLIDER_VELOCITY_'])
            avoid = float(values['_SLIDER_AVOID_'])
            turn = float(values['_SLIDER_TURN_'])

            flock.max_speed = max_speed
            flock.min_speed = min_speed
            flock.turnfactor = turn
            flock.perception = perception
            flock.move_to_middle_strength = centering  # for towards center of mass
            flock.formation_flying_strength = velocity  # for velocity matching
            flock.avoid_strength = avoid
        window.Element('_NUM_BIRDS_').Update(current_num_birds)
        draw(window, flock)

    print(flock.iterations)
    print(flock.polarization())
    window.Close()

if __name__ == '__main__':
    main(200)
