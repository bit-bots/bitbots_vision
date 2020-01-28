#! /usr/bin/env python3

import plotly.graph_objects as go

labels   =  ['FCN_8',               'FCN_8_VGG',           'FCN_8_MOBILENET',       'FCN_32',               'FCN_32_VGG',           'FCN_32_MOBILENET',     'MOBILENET_UNET',       'VGG_UNET',             'UNET_MINI',            'UNET',                 'MOBILENET_SEGNET',     "SEGNET",               'VGG_SEGNET',           'VGG_PSPNET',           'PSPNET',               'BITBOTS_VISION']
accuracy =  [0.09552850965726173,   0.21273195608861029,    0.16679876807045801,    0.09709590336697249,    0.21229216405900858,    0.17042696812732072,    0.08202800070498623,    0.3407829262442508,     0.07141570654292564,    0.10077905722257108,    0.06956342451989987,    0.09374395130717822,    0.16388846588673564,    0.10218388684051859,    0.16388846588673564,    0.011721170167156267]
std      =  [0.001875067012252506,  0.008030933452583594,   0.0027241659757147835,  0.005078963891977667,   0.005593971949960451,   0.006461228332391458,   0.002844587840519984,   0.05627074338242137,    0.0019190096723574028,  0.001876566445583839,   0.003248591561855505,   0.0053748740198840605,  0.009410522012387519,   0.00341287279486953,    0.001583092849885392,   0.0021944406596045426]

fig = go.Figure()
fig.add_trace(go.Bar(
    name='Laufzeit',
    x = labels,
    y = accuracy,
    text= ['%.3f'%round(value, 3) for value in accuracy],
    textposition='outside',
    error_y=dict(type='data', array=std)
))

fig.update_layout(barmode='group')

fig.update_layout(
    title="Laufzeit pro Bild auf Intel Core i7-4810MQ CPU @ 2.80GHz",
    yaxis_title="Laufzeit in s",
    font=dict(
        family="Courier New, monospace",
        size=25,
        color="#7f7f7f"),
    paper_bgcolor='rgba(0,0,0,0)',
    plot_bgcolor='#e5ecf6')

# fig.update_layout(yaxis_type="log")

# fig.show()

fig.write_image("/home/jan/runtime_plot.pdf", width=1500, height=1100)  # Save PDF

