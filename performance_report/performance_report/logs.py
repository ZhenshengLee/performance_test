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

import itertools
import json
import pandas as pd
import os


from .utils import DatasetConfig, DEFAULT_TEST_NAME, ExperimentConfig, ThemeConfig


def parseLog(log_dir: str, test_name: str, experiment: ExperimentConfig):
    """Load logfile into header dictionary and pandas dataframe."""
    header = ''
    dataframe = pd.DataFrame()
    filename = os.path.join(log_dir, test_name, experiment.log_file_name())
    try:
        with open(filename) as source:
            try:
                header = json.load(source)
            except json.decoder.JSONDecodeError:
                print("Unable to decode JSON file " + filename)
            dataframe = pd.json_normalize(header, 'analysis_results')
            if not dataframe.empty:
                del header['analysis_results']
                dataframe['latency_min_ms'] = dataframe['latency_min'] * 1000
                dataframe['latency_max_ms'] = dataframe['latency_max'] * 1000
                dataframe['latency_mean_ms'] = dataframe['latency_mean'] * 1000
                dataframe['latency_variance_ms'] = dataframe['latency_variance'] * 1000 * 1000
                dataframe['latency_M2_ms'] = dataframe['latency_M2'] * 1000 * 1000
                dataframe['cpu_usage_percent'] = dataframe['cpu_info_cpu_usage']
                dataframe['ru_maxrss'] = dataframe['sys_tracker_ru_maxrss']
                dataframe['T_experiment'] = dataframe['experiment_start'] / 1000000000
                # get experiement settings as dataframe
                exp_df = experiment.as_dataframe()
                exp_df = exp_df.loc[exp_df.index.repeat(len(dataframe.index))].reset_index()
                dataframe = pd.concat([exp_df, dataframe], axis=1)
    except FileNotFoundError:
        print("Results for experiment " + filename + " do not exist")
        raise FileNotFoundError()
    return header, dataframe


def getExperiments(yaml_experiments: list) -> 'list[ExperimentConfig]':
    experiments = []
    for experiment in yaml_experiments:
        config_matrix = coerce_dict_vals_to_lists(experiment)
        config_combos = config_cartesian_product(config_matrix)
        experiment_configs = [ExperimentConfig(**args) for args in config_combos]
        for config in experiment_configs:
            if config not in experiments:
                experiments.append(config)
    return experiments


def coerce_to_list(val_or_list: object) -> list:
    if isinstance(val_or_list, list):
        return val_or_list
    return [val_or_list]


def coerce_dict_vals_to_lists(d: dict) -> dict:
    accum = {}
    for k in d:
        accum[k] = coerce_to_list(d[k])
    return accum


def config_cartesian_product(d: dict) -> list:
    accum = []
    keys, values = zip(*d.items())
    for bundle in itertools.product(*values):
        accum.append(dict(zip(keys, bundle)))
    return accum


def getDatasets(yaml_datasets: dict, log_dir) -> dict:
    datasets = {}
    for dataset_id, dataset_details in yaml_datasets.items():
        test_name = dataset_details.get('test_name', DEFAULT_TEST_NAME)
        experiments = getExperiments(dataset_details['experiments'])
        dataframes = []
        headers = []
        for experiment in experiments:
            try:
                header, dataframe = parseLog(log_dir, test_name, experiment)
                headers.append(header)
                dataframes.append(dataframe)
            except FileNotFoundError:
                pass
        # concate all dfs to one single one
        if not headers:
            print(f"Skipping dataset {dataset_id} due to no available experiments")
            continue
        results_df = pd.concat(dataframes, ignore_index=True)
        config_matrix = coerce_dict_vals_to_lists(dataset_details)
        config_combos = config_cartesian_product(config_matrix)
        for cfg in config_combos:
            theme_matrix = coerce_dict_vals_to_lists(cfg['theme'])
            theme_combos = config_cartesian_product(theme_matrix)
            themes = [ThemeConfig(**args) for args in theme_combos]
            for theme in themes:
                dataset = DatasetConfig(
                    name=cfg['name'],
                    theme=theme,
                    experiments=experiments,
                    headers=headers,
                    dataframe=results_df)
                datasets[dataset_id] = dataset
    return datasets
