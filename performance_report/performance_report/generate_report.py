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

from bokeh.embed import components
import bokeh.util.version

from jinja2 import Environment, FileSystemLoader, select_autoescape
from .figures import generateFigure
from .logs import getDatasets, coerce_to_list
from .utils import PerfArgParser


def generateReports(report_cfg_file, log_dir):
    cfg_dir, _ = os.path.split(report_cfg_file)
    html_figures = {}
    with open(report_cfg_file, "r") as f:
        reports_cfg = yaml.load(f, Loader=yaml.FullLoader)
        datasets = getDatasets(reports_cfg["datasets"], log_dir)
        try:
            for report_name, report_cfg in reports_cfg['reports'].items():
                for fig in report_cfg['figures']:
                    plot = generateFigure(fig, datasets)
                    script, div = components(plot)
                    html_figures[fig['name']] = script + div
                # add current bokeh CDN version to template variables
                html_figures['bokeh_version'] = bokeh.util.version.base_version()
                # fill in templates
                for template in coerce_to_list(report_cfg['template_name']):
                    template_dir, template_file = os.path.split(template)
                    loader_dir = os.path.join(cfg_dir, template_dir)
                    env = Environment(
                        loader=FileSystemLoader(loader_dir),
                        autoescape=select_autoescape()
                    )
                    template = env.get_template(template_file)
                    # output should match input file extension to support .md and .html reports
                    template_file_extension = template_file.split('.')[-1]
                    report_title = report_cfg['report_title']
                    output = template.render(html_figures, title=report_title)
                    output_file = \
                        os.path.join(log_dir, report_name + '.' + template_file_extension)
                    with open(output_file, 'w') as result:
                        result.write(output)
        except KeyError:
            print("Oops, something is wrong with the provided"
                  "report configuration file....exiting")


def main():
    parser = PerfArgParser()
    parser.init_args()
    args = parser.parse_args()
    log_dir = getattr(args, "log_dir")
    report_cfg_files = getattr(args, "configs")

    for report_cfg_file in report_cfg_files:
        generateReports(report_cfg_file, log_dir)


if __name__ == "__main__":
    main()
