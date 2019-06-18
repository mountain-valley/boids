import numpy as np
from boid import Boid
# import PySimpleGUIWeb as sg
import PySimpleGUI as sg

width = 1000
height = 1000
starting_num_birds = 40


def draw(window, flock):
    for boid in flock:
        boid.edges()
        boid.apply_behaviour(flock)
        boid.update()
        boid.show(window)


def main():
    flock = [Boid(*np.random.rand(2)*1000, width, height) for _ in range(starting_num_birds)]   # type: Boid:list
    current_num_birds = starting_num_birds

    layout =    [[sg.Text('Boid Flocking'), sg.Text('Number Birds = '), sg.Text('', size=(4,1), key='_NUM_BIRDS_')],
                [sg.Graph((width,height), (0,0), (width,height), background_color='GhostWhite', key='_GRAPH_')],
                [sg.T('Number of birds'), sg.Slider(range=(4,80),orientation='h', default_value=starting_num_birds, key='_SLIDER_', enable_events=True),
                sg.T('Max Force (.3)'), sg.Slider(range=(0,1), default_value=.3,  resolution=.1, orientation='h',  key='_SLIDER_FORCE_', enable_events=True)],
                [sg.T('Max Speed (5)'), sg.Slider(range=(1,30), default_value=5, orientation='h', key='_SLIDER_SPEED_', enable_events=True),
                sg.T('Perception (100)'), sg.Slider(range=(0,200),default_value=100,  orientation='h', key='_SLIDER_PERCEPTION_', enable_events=True)],
                 [sg.Exit()],]

    window = sg.Window('Boids', layout)
    graph = window.Element('_GRAPH_')               # type: sg.Graph
    while True:
        event, values = window.Read(timeout=0)
        if event in (None, 'Exit'):
            break
        if event == '_SLIDER_':
            num_birds = int(values['_SLIDER_'])
            if num_birds > current_num_birds:
                flock = flock + [Boid(*np.random.rand(2)*1000, width, height) for _ in range(num_birds-current_num_birds)]
            else:
                for i in range(current_num_birds-num_birds):
                    graph.DeleteFigure(flock[-1].drawing_id)
                    del flock[-1]
            current_num_birds = num_birds
        elif event.startswith('_SLIDER_'):
            max_force = float(values['_SLIDER_FORCE_'])
            max_speed = int(values['_SLIDER_SPEED_'])
            perception = int(values['_SLIDER_PERCEPTION_'])
            for bird in flock:
                bird = bird  # type: boid.Boid
                bird.max_force = max_force
                bird.max_speed = max_speed
                bird.perception = perception
        window.Element('_NUM_BIRDS_').Update(current_num_birds)
        draw(window, flock)


    window.Close()

if __name__ == '__main__':
    main()
