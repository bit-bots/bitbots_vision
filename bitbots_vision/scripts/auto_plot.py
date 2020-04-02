#! /usr/bin/env python3

import os
import cv2
import numpy as np
import yaml
import statistics
import colorsys
import plotly.offline as py
import plotly.graph_objs as go

# Config
########

const_vision_accuracy = 0.925
const_vision_color = 'red'
accuracy_evaluation_file = "/home/jan/accuracy_eval.yaml"
max_epochs = 60
select_data = "mean_IU"
percentile_rank = 70
main_plot_filename = "/tmp/accuracy_plot"
selected_plot_filename = "/tmp/accuracy_sel_plot"
mean_acc_plot_filename = "/tmp/mean_accuracy_plot"
encoder_plot_filename = "/tmp/encoder_plot"
decoder_plot_filename = "/tmp/decoder_plot"

timing_evaluation_file = "/home/jan/timing_eval_nuc5.yaml"
timing_plot_filename = "/tmp/timing_plot"
const_vision_timing = 0.011721170167156267
const_vision_timing_std_dev = 0.0021944406596045426

performance_plot_filename = "/tmp/performance_plot"

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
    "resnet50",
]

decoders = [
    "fcn_8",
    "fcn_32",
    "unet",
    "segnet",
    "pspnet",
]

selected_models = [
    # "fcn_8",
    # "fcn_32",
    # "fcn_8_vgg",
    # "fcn_32_vgg",
    "fcn_8_resnet50",
    "fcn_32_resnet50",
    "fcn_8_mobilenet",
    "fcn_32_mobilenet",
    # "pspnet",
    # "vgg_pspnet",
    # "resnet50_pspnet",
    # "pspnet_50",
    # "pspnet_101",
    # "unet_mini",
    # "unet",
    # "vgg_unet",
    "resnet50_unet",
    "mobilenet_unet",
    # "segnet",
    # "vgg_segnet",
    "resnet50_segnet",
    "mobilenet_segnet",
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
            y=[const_vision_accuracy]*max_epochs,
            mode='lines',
            name='BITBOTS_VISION',
            line=dict(color=const_vision_color, width=5)))

    fig.update_layout(
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
    models_data = {}
    for model in models:
        models_data[model] = [evaluation_data["{}.{}".format(model, i)][select_data] for i in range(max_epochs)]

    fig = go.Figure()

    # Plot vision data
    fig.add_bar(
        x=['BITBOTS_VISION'],
        y=[const_vision_accuracy],
        text=['%.3f'%round(const_vision_accuracy, 3)],
        marker_color=[const_vision_color],
        textposition='auto',
    )

    # Plot evaluation data
    for model, data in models_data.items():
        percentile = sorted(data)[-(int(((100-percentile_rank)/100)*len(data))):]
        datapoint = statistics.mean(percentile)  # mean of percentile
        # datapoint = statistics.mean(data)  # mean
        fig.add_bar(
            x=[model.upper()],
            y=[datapoint],
            text=['%.3f'%round(datapoint, 3)],
            marker_color=[colors[main_models.index(model)]],
            textposition='auto',
            error_y=dict(type='constant', value=statistics.stdev(percentile)),
            # error_y=dict(type='constant', value=statistics.stdev(data)),
        )

    fig.update_layout(
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='#e5ecf6',
            showlegend=False)

    fig.update_xaxes(title_text="Model", gridwidth=1)
    fig.update_yaxes(title_text="Mean accuracy", gridwidth=1)
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
            y=[const_vision_accuracy]*max_epochs,
            mode='lines',
            name='BITBOTS_VISION',
            line=dict(color=const_vision_color, width=5)))

    fig.update_layout(
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='#e5ecf6')

    fig.update_xaxes(title_text="Epoch", gridwidth=1)
    fig.update_yaxes(title_text="Accuracy", gridwidth=1)
    return fig

def get_timing_plot(evaluation_data, models, colors):
    fig = go.Figure()

    # Plot vision data
    fig.add_bar(
        x=['BITBOTS_VISION'],
        y=[const_vision_timing],
        text=['%.3f'%round(const_vision_timing, 3)],
        marker_color=[const_vision_color],
        textposition='auto',
        error_y=dict(type='constant', value=const_vision_timing_std_dev),
    )

    # Plot evaluation data
    for model, data in evaluation_data.items():
        fig.add_bar(
            x=[model.upper()],
            y=[data['mean']],
            text=['%.3f'%round(data['mean'], 3)],
            marker_color=[colors[main_models.index(model)]],
            textposition='auto',
            error_y=dict(type='constant', value=data['std']),
        )

    fig.update_layout(
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='#e5ecf6',
            showlegend=False)

    fig.update_xaxes(title_text="Model", gridwidth=1)
    fig.update_yaxes(title_text="Time [s]", gridwidth=1)
    return fig

def get_performance_plot(accuracy_data, timing_data, models, colors):
    # Extract model-specific data
    models_data = {}
    for model in models:
        models_data[model] = [accuracy_data["{}.{}".format(model, i)][select_data] for i in range(max_epochs)]

    fig = go.Figure()

    # Plot vision data
    fig.add_bar(
        x=['BITBOTS_VISION'],
        y=[const_vision_accuracy / const_vision_timing],
        text=['%.3f'%round(const_vision_accuracy / const_vision_timing, 3)],
        marker_color=[const_vision_color],
        textposition='auto',
    )

    # Plot evaluation data
    for model, data in models_data.items():
        percentile = sorted(data)[-(int(((100-percentile_rank)/100)*len(data))):]
        datapoint = statistics.mean(percentile) / timing_evaluation_data[model]['mean']  # mean of percentile per time
        fig.add_bar(
            x=[model.upper()],
            y=[datapoint],
            text=['%.3f'%round(datapoint, 3)],
            marker_color=[colors[models.index(model)]],
            textposition='auto',
        )

    fig.update_layout(
            font=dict(
                    family="Courier New, monospace",
                    size=25,
                    color="#7f7f7f"),
            paper_bgcolor='rgba(0,0,0,0)',
            plot_bgcolor='#e5ecf6',
            showlegend=False)

    fig.update_xaxes(title_text="Model", gridwidth=1)
    fig.update_yaxes(title_text="Performance [accuracy/s]", gridwidth=1)
    return fig


# Load accuracy evaluation data
with open(accuracy_evaluation_file, 'r') as file:
    accuracy_evaluation_data = yaml.full_load(file)

# Load timing evaluation data
with open(timing_evaluation_file, 'r') as file:
    timing_evaluation_data = yaml.full_load(file)

# Create plot with all models
main_fig = get_line_plot(accuracy_evaluation_data, main_models, get_N_HexCol(len(main_models)))
#main_fig.write_image(main_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(main_fig, filename=main_plot_filename + ".html", auto_open=True)  # Show HTML

# Create plot with selected models
selected_fig = get_line_plot(accuracy_evaluation_data, selected_models, get_N_HexCol(len(main_models)))
#selected_fig.write_image(selected_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(selected_fig, filename=selected_plot_filename + ".html", auto_open=True)  # Show HTML

# Create plot for each encoder
encoder_fig = get_subnet_plot(accuracy_evaluation_data, encoders, main_models, get_N_HexCol(len(encoders)))
#selected_fig.write_image(selected_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(encoder_fig, filename=encoder_plot_filename + ".html", auto_open=True)  # Show HTML

# Create plot for each encoder
decoder_fig = get_subnet_plot(accuracy_evaluation_data, decoders, main_models, get_N_HexCol(len(decoders)))
#selected_fig.write_image(selected_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(decoder_fig, filename=decoder_plot_filename + ".html", auto_open=True)  # Show HTML

# Create bar plot with mean accuracy of percentile of all models
mean_acc_fig = get_bar_plot(accuracy_evaluation_data, main_models, get_N_HexCol(len(main_models)))
# mean_acc_fig.write_image(mean_acc_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(mean_acc_fig, filename=mean_acc_plot_filename + ".html", auto_open=True)  # Show HTML

# Create timing bar plot with all models
timing_fig = get_timing_plot(timing_evaluation_data, main_models, get_N_HexCol(len(main_models)))
# timing_fig.write_image(timing_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(timing_fig, filename=timing_plot_filename + ".html", auto_open=True)  # Show HTML

# Create performance bar plot with all models
performance_fig = get_performance_plot(accuracy_evaluation_data, timing_evaluation_data, selected_models, get_N_HexCol(len(selected_models)))
# performance_fig.write_image(performance_plot_filename + ".pdf", width=1500, height=1100)  # Save PDF
py.plot(performance_fig, filename=performance_plot_filename + ".html", auto_open=True)  # Show HTML
