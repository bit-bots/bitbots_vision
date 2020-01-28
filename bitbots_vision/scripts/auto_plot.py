#! /usr/bin/env python3

import os
import cv2
import numpy as np
import yaml
import colorsys
import plotly.offline as py
import plotly.graph_objs as go

# Config
########

const_vision_value = 0.925
evaluation_file = "/home/jan/eval.yaml"
max_epochs = 25
select_data = "mean_IU"
main_plot_filename = "/home/jan/accuracy_plot"
selected_plot_filename = "/home/jan/accuracy_sel_plot"

main_models = [
        'fcn_8',
        'fcn_8_vgg',
        'fcn_8_mobilenet',
        'fcn_32',
        'fcn_32_vgg',
        'fcn_32_mobilenet',
        'mobilenet_unet',
        'vgg_unet',
        'unet_mini',
        'unet',
        'segnet',
        'mobilenet_segnet',
        'vgg_segnet',
        'vgg_pspnet',
        'pspnet'
        ]

selected_models = [
        # 'fcn_8',
        # 'fcn_8_vgg',
        # 'fcn_8_mobilenet',
        # 'fcn_32',
        # 'fcn_32_vgg',
        # 'fcn_32_mobilenet',
        # 'mobilenet_unet',
        # 'vgg_unet',
        'unet_mini',
        # 'unet',
        'segnet',
        'mobilenet_segnet',
        # 'vgg_segnet',
        # 'vgg_pspnet',
        # 'pspnet'
        ]

# Generate colors
def get_N_HexCol(N=5):
    HSV_tuples = [(x * 1.0 / N, 1, 1) for x in range(N)]
    hex_out = []
    for rgb in HSV_tuples:
        rgb = map(lambda x: int(x * 255), colorsys.hsv_to_rgb(*rgb))
        hex_out.append('#%02x%02x%02x' % tuple(rgb))
    return hex_out

def get_plot(evaluation_data, models, colors):
    # Extract model-specific data
    models_data = {}
    for model in models:
        models_data[model] = [evaluation_data["{}.{}".format(model, i)] for i in range(max_epochs)]

    # Plot evaluation data
    # Create traces
    fig = go.Figure()
    for model, data in models_data.items():
        fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[datapoint[select_data] for datapoint in data],
            mode='lines',
            name=model.upper(),
            line=dict(width=3, color=colors[model.__hash__() % len(evaluation_data)])))

    fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[const_vision_value]*max_epochs,
            mode='lines',
            name='bitbots_vision',
            line=dict(color='red', width=5)))

    fig.update_layout(
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='rgba(0,0,0,0)')

    fig.update_xaxes(title_text="Epoch", gridwidth=1)
    fig.update_yaxes(title_text="Accuracy", gridwidth=1)
    return fig


# Load evaluation data
with open(evaluation_file, 'r') as file:
    evaluation_data = yaml.full_load(file)

colors = get_N_HexCol(len(evaluation_data))

# Create plot with all models
main_fig = get_plot(evaluation_data, main_models, colors)
main_fig.write_image(main_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
# py.plot(main_fig, filename=main_plot_filename + ".html", auto_open=True)  # Show HTML

# Create plot with selected models
selected_fig = get_plot(evaluation_data, selected_models, colors)
selected_fig.write_image(selected_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
# py.plot(selected_fig, filename=selected_plot_filename + ".html", auto_open=True)  # Show HTML
