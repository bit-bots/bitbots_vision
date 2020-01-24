#! /usr/bin/env python3

import os
import cv2
import numpy as np
import yaml
import plotly.offline as py
import plotly.graph_objs as go

# Config
########

const_vision_value = 0.925
evaluation_file = "/home/jan/eval.yaml"
max_epochs = 25
select_data = "mean_IU"
plotly_filename = "accuracy_plot.html"

models = [
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

choosen_models = [
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
        # 'segnet',
        'mobilenet_segnet',
        'vgg_segnet',
        # 'vgg_pspnet',
        # 'pspnet'
        ]


# Load evaluation data
with open(evaluation_file, 'r') as file:
    evaluation_data = yaml.full_load(file)

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
        name=model.upper()))

fig.add_trace(go.Scatter(
        x=list(range(max_epochs)),
        y=[const_vision_value]*max_epochs,
        mode='lines',
        name='bitbots_vision',
        line = dict(color='red', width=5)))

fig.update_layout(
        font=dict(
                family="Courier New, monospace",
                size=25,
                color="#7f7f7f"),
        paper_bgcolor='rgba(0,0,0,0)',
        plot_bgcolor='rgba(0,0,0,0)')

fig.update_xaxes(title_text="Epoch", gridwidth=1)
fig.update_yaxes(title_text="Accuracy", gridwidth=1)

# fig.write_image("~/accuracy.pdf")


py.plot(fig, filename=plotly_filename, auto_open=True)
