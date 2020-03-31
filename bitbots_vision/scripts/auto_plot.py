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
main_plot_filename = "/tmp/accuracy_plot"
selected_plot_filename = "/tmp/accuracy_sel_plot"
max_plot_filename = "/home/jan/max_accuracy_plot"
encoder_plot_filename = "/tmp/encoder_plot"
decoder_plot_filename = "/tmp/decoder_plot"

main_models = [
    "fcn_8",
    "fcn_32",
    "fcn_8_vgg",
    "fcn_32_vgg",
    "fcn_8_resnet50",
    "fcn_32_resnet50",
    "fcn_8_mobilenet",
    "fcn_32_mobilenet",
    "pspnet",
    "vgg_pspnet",
    "resnet50_pspnet",
    "pspnet_50",
    "pspnet_101",
    "unet_mini",
    "unet",
    "vgg_unet",
    "resnet50_unet",
    "mobilenet_unet",
    "segnet",
    "vgg_segnet",
    "resnet50_segnet",
    "mobilenet_segnet",
    ]

encoders = [
    "mobilenet",
    "vgg",
    "vanilla",
]

decoders = [
    "fcn_8",
    "fcn_32",
    "unet",
    "segnet",
    "pspnet"
]

selected_models = [
    # "fcn_8",
    # "fcn_32",
    # "fcn_8_vgg",
    # "fcn_32_vgg",
    # "fcn_8_resnet50",
    # "fcn_32_resnet50",
    # "fcn_8_mobilenet",
    # "fcn_32_mobilenet",
    # "pspnet",
    # "vgg_pspnet",
    # "resnet50_pspnet",
    # "pspnet_50",
    # "pspnet_101",
    # "unet_mini",
    # "unet",
    # "vgg_unet",
    # "resnet50_unet",
    # "mobilenet_unet",
    # "segnet",
    # "vgg_segnet",
    # "resnet50_segnet",
    # "mobilenet_segnet",
    ]

# Generate colors
def get_N_HexCol(N=5):
    HSV_tuples = [(x * 1.0 / N, 1, 1) for x in range(N)]
    hex_out = []
    for rgb in HSV_tuples:
        rgb = map(lambda x: int(x * 255), colorsys.hsv_to_rgb(*rgb))
        hex_out.append('#%02x%02x%02x' % tuple(rgb))
    return hex_out

def get_line_plot(evaluation_data, models, colors):
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
            line=dict(width=3, color=colors[main_models.index(model)])))

    fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[const_vision_value]*max_epochs,
            mode='lines',
            name='BITBOTS_VISION',
            line=dict(color='red', width=5)))

    fig.update_layout(
            title="Accuracy on evaluation dataset per epoch",
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='#e5ecf6')

    fig.update_xaxes(title_text="Epoch", gridwidth=1)
    fig.update_yaxes(title_text="Accuracy", gridwidth=1)
    return fig

def get_bar_plot(evaluation_data, models, colors):
    # Extract model-specific data
    data = [const_vision_value]
    model_names = ['BITBOTS_VISION']
    models_data = [[const_vision_value]]
    for model in models:
        data.append(max([evaluation_data["{}.{}".format(model, i)][select_data] for i in range(max_epochs)]))
        model_names.append(model.upper())
        models_data.append([evaluation_data["{}.{}".format(model, i)][select_data] for i in range(max_epochs)])

    print(models_data)

    # Plot evaluation data
    fig = go.Figure(data=[go.Bar(
            x=model_names,
            y=data,
            text=model_names,
            textposition='auto',
            marker_color=colors,
            error_y=dict(type='data', array=models_data),
        )])

    fig.update_layout(
            title="Max Accuracy on evaluation dataset per model",
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='#e5ecf6')

    fig.update_xaxes(title_text="Model", gridwidth=1)
    fig.update_yaxes(title_text="Accuracy", gridwidth=1)
    return fig

def get_subnet_plot(evaluation_data, subnets, models, colors):
    # Extract model-specific data
    models_data = {}
    for model in models:
        models_data[model] = [evaluation_data["{}.{}".format(model, i)] for i in range(max_epochs)]

    performance_per_subnet = {}
    for subnet in subnets:
        performance_per_subnet[subnet] = list()

    for epoch in range(max_epochs):
        subnet_value_list = {}

        for subnet in subnets:
            subnet_value_list[subnet] = list()

        for model in models:
            corresponding_subnet = "vanilla"
            for subnet in subnets:
                if subnet in model:
                    corresponding_subnet = subnet

            subnet_value_list[corresponding_subnet].append(evaluation_data["{}.{}".format(model, epoch)][select_data])

        for subnet in subnets:
            performance_per_subnet[subnet].append(np.array(subnet_value_list[subnet]).mean())


    # Plot evaluation data
    # Create traces
    fig = go.Figure()
    for model, data in performance_per_subnet.items():
        fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[datapoint for datapoint in data],
            mode='lines',
            name=model.upper(),
            line=dict(width=3, color=colors[list(performance_per_subnet.keys()).index(model)])))

    fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[const_vision_value]*max_epochs,
            mode='lines',
            name='BITBOTS_VISION',
            line=dict(color='red', width=5)))

    fig.update_layout(
            title="Genauigkeit auf Evaluations-Datensatz pro Epoche",
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='#e5ecf6')

    fig.update_xaxes(title_text="Epoche", gridwidth=1)
    fig.update_yaxes(title_text="Genauigkeit", gridwidth=1)
    return fig

# Load evaluation data
with open(evaluation_file, 'r') as file:
    evaluation_data = yaml.full_load(file)

colors = get_N_HexCol(len(main_models)+1)

# Create plot with all models
main_fig = get_line_plot(evaluation_data, main_models, colors)
#main_fig.write_image(main_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(main_fig, filename=main_plot_filename + ".html", auto_open=True)  # Show HTML

# Create plot with selected models
selected_fig = get_line_plot(evaluation_data, selected_models, colors)
#selected_fig.write_image(selected_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(selected_fig, filename=selected_plot_filename + ".html", auto_open=True)  # Show HTML

# Create plot for each encoder
encoder_fig = get_subnet_plot(evaluation_data, encoders, main_models, colors)
#selected_fig.write_image(selected_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(encoder_fig, filename=encoder_plot_filename + ".html", auto_open=True)  # Show HTML

# Create plot for each encoder
decoder_fig = get_subnet_plot(evaluation_data, decoders, main_models, colors)
#selected_fig.write_image(selected_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(decoder_fig, filename=decoder_plot_filename + ".html", auto_open=True)  # Show HTML

# Create bar plot with all models
max_fig = get_bar_plot(evaluation_data, main_models, colors)
# max_fig.write_image(max_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(max_fig, filename=max_plot_filename + ".html", auto_open=True)  # Show HTML
