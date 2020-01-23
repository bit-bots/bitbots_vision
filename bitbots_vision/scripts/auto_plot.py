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
select_model = "fcn_32"
select_data = "mean_IU"
plotly_filename = "plotly_{}.html".format(select_model)


# Load evaluation data
with open(evaluation_file, 'r') as file:
    evaluation_data = yaml.full_load(file)

# Select model
model_data = []
for i in range(max_epochs):
    model_data.append(evaluation_data["{}.{}".format(select_model, i)])

# Select data group
data_group = []
for data in model_data:
    data_group.append(data[select_data])

# Plot evaluation data
# Create traces
fig = go.Figure()
fig.add_trace(go.Scatter(
        x=list(range(len(data_group))),
        y=data_group,
        mode='lines',
        name=select_model))
fig.add_trace(go.Scatter(
        x=list(range(len(data_group))),
        y=[const_vision_value]*len(data_group),
        mode='lines',
        name='const_vision'))

py.plot(fig, filename=plotly_filename, auto_open=True)
