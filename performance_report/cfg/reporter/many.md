<script type="text/javascript"
        src="https://cdn.bokeh.org/bokeh/release/bokeh-{{ bokeh_version|safe }}.min.js"></script>

# {{ title }}

This is an example performance report generated to markdown.

#### Compare latency for different transports

{{ compare_transport|safe }}

#### Compare CPU usage for different transports

{{ compare_transport_cpu_usage|safe }}

#### Compare latency vs msg size for different transports

{{ compare_msg_size|safe }}

#### Compare latency vs msg size for different number of subscribers

{{ compare_num_subs|safe }}