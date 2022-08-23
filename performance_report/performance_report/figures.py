# Copyright 2022 Apex.AI, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
import pandas as pd

from bokeh.models import ColumnDataSource, FactorRange
from bokeh.models.tools import HoverTool
from bokeh.plotting import figure

from .utils import DatasetConfig

result = {
    'x': [],
    'y_lat': [],
    'y_cpu': []
}


def generateFigure(figConfig, datasets: "list[DatasetConfig]"):
    # time series, normal range
    fig = None
    is_categorical = False
    if figConfig['x_range'] == 'T_experiment':
        fig = figure(
                title=figConfig['title'],
                x_axis_label=figConfig['x_axis_label'],
                y_axis_label=figConfig['y_axis_label'],
                plot_width=figConfig['size']['width'],
                plot_height=figConfig['size']['height'],
                margin=(10, 10, 10, 10)
        )
    elif figConfig['x_range'] == 'Experiment' and figConfig['y_range'] == 'latency_mean':
        return _generate_box_and_whiskers(figConfig, datasets)
    else:
        # assume categorical if not a time series
        is_categorical = True
        fig = figure(
                name=figConfig['name'],
                title=figConfig['title'],
                x_axis_label=figConfig['x_axis_label'],
                y_axis_label=figConfig['y_axis_label'],
                x_range=FactorRange(),
                plot_width=figConfig['size']['width'],
                plot_height=figConfig['size']['height'],
                margin=(10, 10, 10, 10)
        )
    for dataset_name in figConfig['datasets']:
        if dataset_name not in datasets:
            print(f"dataset {dataset_name} missing, skipping for {figConfig['name']}")
            continue
        dataset = datasets[dataset_name]
        df = dataset.dataframe

        # filter dataframe based on specified ranges
        if(len(dataset.experiments) > 1):
            # if multiple experiments in dataset
            filtered_results = []
            for experiment in dataset.experiments:
                exp_df = experiment.as_dataframe()
                exp_cols = list(exp_df.columns.values)
                matching_rows = \
                    dataset.dataframe[exp_cols].stack().isin(exp_df.stack().values).unstack()
                result_df = dataset.dataframe[matching_rows.all(axis='columns')]
                # default to average of specified range
                summary_df = result_df.groupby(experiment.get_members()).mean().reset_index()
                filtered_results.append(summary_df)
            df = pd.concat(filtered_results, ignore_index=True)
        line_name = dataset.name
        scatter_name = line_name + ' ' + dataset.theme.marker.shape
        if is_categorical:
            fig.x_range.factors = list(df[figConfig['x_range']])
            source = ColumnDataSource(df)
            fig.scatter(
                name=scatter_name,
                x=figConfig['x_range'],
                y=figConfig['y_range'],
                source=source,
                marker=dataset.theme.marker.shape,
                size=dataset.theme.marker.size,
                fill_color=dataset.theme.color
            )
            fig.line(
                name=line_name,
                x=figConfig['x_range'],
                y=figConfig['y_range'],
                source=source,
                line_color=dataset.theme.color,
                line_dash=dataset.theme.line.style,
                line_width=dataset.theme.line.width,
                line_alpha=dataset.theme.line.alpha,
                legend_label=line_name,
            )
        else:
            fig.scatter(
                name=scatter_name,
                x=figConfig['x_range'],
                y=figConfig['y_range'],
                source=df,
                marker=dataset.theme.marker.shape,
                size=dataset.theme.marker.size,
                fill_color=dataset.theme.color
            )
            fig.line(
                name=line_name,
                x=figConfig['x_range'],
                y=figConfig['y_range'],
                source=df,
                line_color=dataset.theme.color,
                line_dash=dataset.theme.line.style,
                line_width=dataset.theme.line.width,
                line_alpha=dataset.theme.line.alpha,
                legend_label=line_name,
            )
        # add hover tool
        hover = HoverTool()
        hover.tooltips = [
            (figConfig['y_axis_label'], '@{' + figConfig['y_range'] + '}{0.0000}'),
        ]
        fig.add_tools(hover)
    return fig


def _generate_box_and_whiskers(figConfig, datasets: "list[DatasetConfig]"):
    df_dict = {
        'name': [],
        'fill_color': [],
        'whisker_bottom': [],
        'mean': [],
        'whisker_top': [],
        'std_dev': [],
        'box_top': [],
        'box_bottom': []
    }

    for dataset_name in figConfig['datasets']:
        if dataset_name not in datasets:
            print(f"dataset {dataset_name} missing, skipping for {figConfig['name']}")
            continue
        dataset = datasets[dataset_name]
        df = dataset.dataframe
        df_dict['name'].append(dataset.name)
        df_dict['fill_color'].append(dataset.theme.color)
        df_dict['whisker_bottom'].append(df['latency_min'].min())
        m = df['latency_mean'].mean()
        df_dict['mean'].append(m)
        df_dict['whisker_top'].append(df['latency_max'].max())
        std_dev = math.sqrt(_combined_variance(df))
        df_dict['std_dev'].append(std_dev)
        df_dict['box_top'].append(m + std_dev)
        df_dict['box_bottom'].append(m - std_dev)

    df = pd.DataFrame.from_dict(df_dict)
    data_source = ColumnDataSource(df)

    fig = figure(
        title=figConfig['title'],
        x_axis_label=figConfig['x_axis_label'],
        y_axis_label=figConfig['y_axis_label'],
        x_range=df['name'],
        plot_width=figConfig['size']['width'],
        plot_height=figConfig['size']['height'],
        margin=(10, 10, 10, 10)
    )
    # lower stem
    fig.segment(
        'name', 'box_bottom', 'name', 'whisker_bottom', color='black', line_width=2,
        source=data_source)
    # upper stem
    fig.segment(
        'name', 'box_top', 'name', 'whisker_top', color='black', line_width=2,
        source=data_source)
    # box
    fig.vbar(
        width=0.2,
        x='name',
        top='box_top',
        bottom='box_bottom',
        line_color='black',
        source=data_source,
        fill_color='fill_color'
    )
    # lower whisker
    fig.scatter(
        size=25,
        x='name',
        y='whisker_top',
        source=data_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color='black'
    )
    # upper whisker
    fig.scatter(
        size=25,
        x='name',
        y='whisker_bottom',
        source=data_source,
        line_color='black',
        line_width=2,
        marker='dash',
        fill_color='black'
    )
    fig.y_range.start = 0
    fig.x_range.range_padding = 0.1
    fig.xaxis.major_label_orientation = math.pi / 8

    return fig


def _combined_variance(df):
    n = df['rate']
    m = df['latency_mean']
    q = (n - 1) * df['latency_variance'] + n * m * m
    nc = n.sum()
    mc = m.mean()
    qc = q.sum()
    return (qc - nc * mc * mc) / (nc - 1)
