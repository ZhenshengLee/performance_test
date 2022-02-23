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

import os
import yaml

from bokeh.io import export_png

from .logs import getDatasets
from .figures import generateFigure
from .utils import PerfArgParser


def generatePlots(plot_cfg_file, log_dir):
    with open(plot_cfg_file, "r") as f:
        plots_cfg = yaml.load(f, Loader=yaml.FullLoader)
        datasets = getDatasets(plots_cfg["datasets"], log_dir)
        try:
            for plot in plots_cfg["plots"]:
                fig = generateFigure(plot, datasets)
                output_file = os.path.join(log_dir, plot['name'] + '.png')
                export_png(fig, filename=output_file)
        except KeyError:
            print("No msg_size_vs_latency_cpu plots specified in given trace yaml file")


def main():
    parser = PerfArgParser()
    parser.init_args()
    args = parser.parse_args()
    log_dir = getattr(args, "log_dir")
    plot_cfg_files = getattr(args, "configs")

    for plot_cfg_file in plot_cfg_files:
        generatePlots(plot_cfg_file, log_dir)


if __name__ == "__main__":
    main()
