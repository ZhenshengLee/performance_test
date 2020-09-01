import itertools
import pandas


def load_logfile(filename):
    """Load logfile into header dictionary and pandas dataframe."""
    with open(filename) as source:
        header = {}
        for item in itertools.takewhile(lambda x: not x.startswith('---'), source):
            if not item.strip():  # Don't care about whitespace-only lines
                continue
            try:
                key = item.split(':')[0].strip()
                value = item.split(':', maxsplit=1)[1].strip()
                header[key] = value
            except Exception:
                print('Error trying to parse header line "{}"'.format(item))
                raise
        dataframe = pandas.read_csv(source, sep='[ \t]*,[ \t]*', engine='python')
        unnamed = [col for col in dataframe.keys() if col.startswith('Unnamed: ')]
        if unnamed:
            dataframe.drop(unnamed, axis=1, inplace=True)
    return header, dataframe


def load_logfiles(logfiles):
    """Load logfiles into header dictionaries and pandas dataframes."""
    headers = []
    dataframes = []

    for logfile in logfiles.value:
        header, dataframe = load_logfile(logfile)
        headers.append(header)
        dataframes.append(dataframe)

    return headers, dataframes
