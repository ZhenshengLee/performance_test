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
