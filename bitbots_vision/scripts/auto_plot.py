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

PDF = True
const_vision_optimized_accuracy = 0.925
const_vision_default_accuracy = 0.010140472
const_vision_natural_light_accuracy = 0.7444397868547964
accuracy_evaluation_file = "/home/jan/accuracy_eval.yaml"
natural_light_accuracy_evaluation_file = "/home/jan/deepfield_natural_performance.yaml"
max_epochs = 60
select_data = "class_wise_IU"
percentile_rank = 70
main_plot_filename = "/tmp/accuracy_plot"
selected_plot_filename = "/tmp/accuracy_sel_plot"
mean_acc_plot_filename = "/tmp/mean_accuracy_plot"
natural_light_mean_acc_plot_filename = "/tmp/natural_light_mean_accuracy_plot"
encoder_plot_filename = "/tmp/encoder_plot"
decoder_plot_filename = "/tmp/decoder_plot"
encoder_decoder_plot_filename = "/tmp/encoder_decoder_plot"

timing_evaluation_file = "/home/jan/timing_eval_nuc5.yaml"
timing_plot_filename = "/tmp/timing_plot"
const_vision_timing = 0.011721170167156267
const_vision_timing_std_dev = 0.0021944406596045426

cost_benifit_plot_filename = "/tmp/cost_benefit_plot"

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
    "unet_mini",
    "segnet",
    "mobilenet_segnet",
    ]

performant_models = [
    "fcn_8_resnet50",
    "fcn_32_resnet50",
    "fcn_8_mobilenet",
    "fcn_32_mobilenet",
    "resnet50_unet",
    "mobilenet_unet",
    "resnet50_segnet",
    "mobilenet_segnet",
    ]

background_color = 'rgba(0,0,0,0)'
second_color = '#F67280'
main_color = '#083358'

grid_color = '#555555'

font_settings = dict(
        family="Courier New, monospace",
        size=20,
        color="#000000")

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
            y=[datapoint[select_data][1] for datapoint in data],
            mode='lines',
            name=model.upper(),
            line=dict(width=5, color=colors[main_models.index(model)])))

    fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[const_vision_optimized_accuracy]*max_epochs,
            mode='lines',
            name='BITBOTS_OPTIMIZED',
            line=dict(color=second_color, width=8)))

    fig.update_layout(
            font=font_settings,
            paper_bgcolor=background_color,
            plot_bgcolor=background_color,
            showlegend=False,
            )

    fig.update_xaxes(
        title_text="Epoch",
        gridwidth=1,
        gridcolor=grid_color,
        )

    fig.update_yaxes(
        title_text="Accuracy",
        gridwidth=1,
        gridcolor=grid_color,
        zeroline=True,
        zerolinewidth=2,
        zerolinecolor=grid_color,
        )
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

            subnet_value_list[corresponding_subnet].append(evaluation_data["{}.{}".format(model, epoch)][select_data][1])

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
            line=dict(width=5, color=colors[list(performance_per_subnet.keys()).index(model)])))

    fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[const_vision_optimized_accuracy]*max_epochs,
            mode='lines',
            name='BITBOTS_OPTIMIZED',
            line=dict(color=second_color, width=8)))

    fig.update_layout(
            font=font_settings,
            paper_bgcolor=background_color,
            plot_bgcolor=background_color,
            showlegend=False,
            )

    fig.update_xaxes(
        title_text="Epoch",
        gridwidth=1,
        gridcolor=grid_color,
        )

    fig.update_yaxes(
        title_text="Accuracy",
        gridwidth=1,
        gridcolor=grid_color,
        zeroline=True,
        zerolinewidth=2,
        zerolinecolor=grid_color,
        )

    return fig

def get_encoder_decoder_plot(evaluation_data, encoder, decoder, models, colors):
    fig = go.Figure()

    # Extract model-specific data
    models_data = {}
    for model in models:
        models_data[model] = [evaluation_data["{}.{}".format(model, i)] for i in range(max_epochs)]

    # Encoder
    subnets = encoder
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

            subnet_value_list[corresponding_subnet].append(evaluation_data["{}.{}".format(model, epoch)][select_data[1]])

        for subnet in subnets:
            performance_per_subnet[subnet].append(np.array(subnet_value_list[subnet]).mean())


    # Plot evaluation data
    # Create traces
    for model, data in performance_per_subnet.items():
        fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[datapoint for datapoint in data],
            mode='lines',
            name=model.upper(),
            line=dict(width=5, color=colors[subnets.index(model)])))

    # Decoder
    subnets = decoder
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

            subnet_value_list[corresponding_subnet].append(evaluation_data["{}.{}".format(model, epoch)][select_data][1])

        for subnet in subnets:
            performance_per_subnet[subnet].append(np.array(subnet_value_list[subnet]).mean())


    # Plot evaluation data
    # Create traces
    for model, data in performance_per_subnet.items():
        fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[datapoint for datapoint in data],
            mode='lines',
            name=model.upper(),
            line=dict(width=5, dash='dot', color=colors[subnets.index(model)+len(encoder)])))

    fig.add_trace(go.Scatter(
            x=list(range(max_epochs)),
            y=[const_vision_optimized_accuracy]*max_epochs,
            mode='lines',
            name='BITBOTS_OPTIMIZED',
            line=dict(color=second_color, width=8)))

    fig.update_layout(
            font=font_settings,
            paper_bgcolor=background_color,
            plot_bgcolor=background_color,
            showlegend=False,
            )

    fig.update_xaxes(
        title_text="Epoch",
        gridwidth=1,
        gridcolor=grid_color,
        )

    fig.update_yaxes(
        title_text="Accuracy",
        gridwidth=1,
        gridcolor=grid_color,
        zeroline=True,
        zerolinewidth=2,
        zerolinecolor=grid_color,
        )

    return fig

def get_accuracy_bar_plot(evaluation_data, models):
    # Extract model-specific data
    models_data = {
        'BITBOTS_OPTIMIZED': [const_vision_optimized_accuracy, 0],
        'BITBOTS_DEFAULT': [const_vision_default_accuracy, 0]
        }
    for model in models:
        datapoints = [evaluation_data["{}.{}".format(model, i)][select_data][1] for i in range(max_epochs)]
        percentile = sorted(datapoints)[-(int(((100-percentile_rank)/100)*len(datapoints))):]
        models_data[model] = [statistics.mean(percentile), statistics.stdev(percentile)]

    # Sort by value
    models_data = {k: v for k, v in sorted(models_data.items(), key=lambda item: item[1][0], reverse=True)}

    fig = go.Figure()

    # Plot evaluation data
    for model, data in models_data.items():
        if 'bitbots' in model.lower():
            bar_color = second_color
            error_color = 'rgba(0,0,0,0)'
        else:
            bar_color = main_color
            error_color = second_color

        fig.add_bar(
            x=[data[0]],
            y=[model.upper()],
            text=['%.3f'%round(data[0], 3)],
            marker_color=[bar_color],
            textposition='auto',
            error_x=dict(
                type='constant',
                value=data[1],
                color=error_color,
                ),
            orientation='h',
        )

    fig.update_layout(
        font=font_settings,
        paper_bgcolor=background_color,
        plot_bgcolor=background_color,
        showlegend=False,
        )

    fig.update_xaxes(
        title_text="Mean accuracy",
        range=[0, 1],
        nticks=11,
        gridwidth=1,
        gridcolor=grid_color,
        zeroline=True,
        zerolinewidth=2,
        zerolinecolor=grid_color
        )

    fig.update_yaxes(
        title_text="Model",
        showgrid=False,
        )
    return fig

def get_natural_light_accuracy_bar_plot(evaluation_data, models):
    # Extract model-specific data
    models_data = {
        'BITBOTS_OPTIMIZED': 0.7444397868547964,
        }
    for model in models:
        models_data[model] = evaluation_data[model][select_data][1]

    # Sort by value
    models_data = {k: v for k, v in sorted(models_data.items(), key=lambda item: item[1], reverse=True)}

    fig = go.Figure()

    # Plot evaluation data
    for model, data in models_data.items():
        if 'bitbots' in model.lower():
            bar_color = second_color
            error_color = 'rgba(0,0,0,0)'
        else:
            bar_color = main_color
            error_color = second_color

        fig.add_bar(
            x=[data],
            y=[model.upper()],
            text=['%.3f'%round(data, 3)],
            marker_color=[bar_color],
            textposition='auto',
            orientation='h',
        )

    fig.update_layout(
        font=font_settings,
        paper_bgcolor=background_color,
        plot_bgcolor=background_color,
        showlegend=False,
        )

    fig.update_xaxes(
        title_text="Mean accuracy",
        range=[0, 1],
        nticks=11,
        gridwidth=1,
        gridcolor=grid_color,
        zeroline=True,
        zerolinewidth=2,
        zerolinecolor=grid_color
        )

    fig.update_yaxes(
        title_text="Model",
        showgrid=False,
        )
    return fig

def get_timing_plot(evaluation_data):
    data = evaluation_data.copy()
    # Add bitbots const
    data['BITBOTS_OPTIMIZED'] = {
        'mean': const_vision_timing,
        'std': const_vision_timing_std_dev
    }

    # Sort by value
    data = {k: v for k, v in sorted(data.items(), key=lambda item: item[1]['mean'])}

    fig = go.Figure()

    # Plot evaluation data
    for model, data in data.items():
        if 'bitbots' in model.lower():
            bar_color = second_color
            error_color = 'rgba(0,0,0,0)'
        else:
            bar_color = main_color
            error_color = second_color

        fig.add_bar(
            x=[data['mean']],
            y=[model.upper()],
            text=['%.3f'%round(data['mean'], 3)],
            marker_color=[bar_color],
            textposition='auto',
            error_x=dict(
                type='constant',
                value=data['std'],
                color=error_color,
                ),
        orientation='h',
        )

    fig.update_layout(
            font=font_settings,
            paper_bgcolor=background_color,
            plot_bgcolor=background_color,
            showlegend=False,
            )

    fig.update_xaxes(
        title_text="Time [s]",
        gridwidth=1,
        gridcolor=grid_color,
        zeroline=True,
        zerolinewidth=2,
        zerolinecolor=grid_color
        )

    fig.update_yaxes(
        title_text="Model",
        showgrid=False,
        )
    return fig

def get_cost_benifit_plot(accuracy_data, timing_data, models):
    # Extract model-specific data
    models_data = {
        'BITBOTS_OPTIMIZED': const_vision_timing / const_vision_optimized_accuracy,
        }

    for model in models:
        datapoints = [accuracy_data["{}.{}".format(model, i)][select_data][1] for i in range(max_epochs)]
        percentile = sorted(datapoints)[-(int(((100-percentile_rank)/100)*len(datapoints))):]
        models_data[model] =  timing_evaluation_data[model]['mean'] / statistics.mean(percentile)  # time per mean of percentile

    # Sort by value
    models_data = {k: v for k, v in sorted(models_data.items(), key=lambda item: item[1])}

    fig = go.Figure()

    # Plot evaluation data
    for model, data in models_data.items():
        if 'bitbots' in model.lower():
            bar_color = second_color
        else:
            bar_color = main_color

        fig.add_bar(
            x=[data],
            y=[model.upper()],
            text=['%.3f'%round(data, 3)],
            marker_color=[bar_color],
            textposition='auto',
            orientation='h',
        )

    fig.update_layout(
            font=font_settings,
            paper_bgcolor=background_color,
            plot_bgcolor=background_color,
            showlegend=False,
            )

    fig.update_xaxes(
        title_text="Cost/Benifit ratio [s/IOU]",
        gridwidth=1,
        gridcolor=grid_color,
        zeroline=True,
        zerolinewidth=2,
        zerolinecolor=grid_color,
        )

    fig.update_yaxes(
            title_text="Model",
            showgrid=False,
            )
    return fig


def output(fig, path, pdf=PDF, width=1500, height=1100):
    if pdf:
        fig.write_image(path + ".pdf", width=width, height=height)  # Save PDF
    else:
        py.plot(fig, filename=path + ".html", auto_open=True)  # Show HTML


# Load accuracy evaluation data
with open(accuracy_evaluation_file, 'r') as file:
    accuracy_evaluation_data = yaml.full_load(file)

# Load natural lightaccuracy evaluation data
with open(natural_light_accuracy_evaluation_file, 'r') as file:
    natural_light_accuracy_evaluation_data = yaml.full_load(file)

# Load timing evaluation data
with open(timing_evaluation_file, 'r') as file:
    timing_evaluation_data = yaml.full_load(file)


# # Create plot with all models
# output(get_line_plot(accuracy_evaluation_data, main_models, get_N_HexCol(len(main_models))), main_plot_filename)

# Create plot with selected models
output(get_line_plot(accuracy_evaluation_data, selected_models, get_N_HexCol(len(main_models))), selected_plot_filename)

# Create plot for each encoder
output(get_subnet_plot(accuracy_evaluation_data, encoders, main_models, get_N_HexCol(len(encoders))), encoder_plot_filename)

# # Create plot for each decoder
# output(get_subnet_plot(accuracy_evaluation_data, decoders, main_models, get_N_HexCol(len(decoders))), decoder_plot_filename)

# # Create plot for each encoder and decoder
# output(get_encoder_decoder_plot(accuracy_evaluation_data, encoders, decoders, main_models, get_N_HexCol(len(encoders+decoders))), encoder_decoder_plot_filename)

# Create bar plot with mean accuracy of percentile of all models
output(get_accuracy_bar_plot(accuracy_evaluation_data, main_models), mean_acc_plot_filename, width=1000, height=1000)

# Create bar plot with mean accuracy of natural light for performant models
output(get_natural_light_accuracy_bar_plot(natural_light_accuracy_evaluation_data, performant_models), natural_light_mean_acc_plot_filename, width=1000, height=500)

# Create timing bar plot with all models
output(get_timing_plot(timing_evaluation_data), timing_plot_filename, width=1000, height=1000)

# Create cost benefit bar plot with all models
output(get_cost_benifit_plot(accuracy_evaluation_data, timing_evaluation_data, performant_models), cost_benifit_plot_filename, width=1000, height=500)
