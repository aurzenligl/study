import os
import pytest
from py.xml import html

@pytest.mark.optionalhook
def pytest_html_results_table_header(cells):
    cells.pop()
    cells.insert(-1, html.th('Markers', class_='sortable'))
    cells.insert(-1, html.th('Start', class_='sortable time', col='time'))

@pytest.mark.optionalhook
def pytest_html_results_table_row(report, cells):
    cells.pop()
    cells.insert(-1, html.td(' '.join(report._extras['markers']) or '~ none ~'))
    cells.insert(-1, html.td('%.2f' % report._extras['start'], class_='col-time'))

@pytest.mark.optionalhook
def pytest_html_results_table_html(report, data):
    data[0].append(report._extras['logs'])

@pytest.mark.hookwrapper
def pytest_runtest_makereport(item, call):
    outcome = yield
    add_html_extras(item, call, outcome)

def add_html_extras(item, call, output):
    report = output.get_result()
    report._extras = extras = {}

    config = item.config
    config._first_setup_start = getattr(config, '_first_setup_start', call.start)
    extras['start'] = call.start - config._first_setup_start

    extras['markers'] = sorted(filter(item.get_marker, item.keywords.keys()))

    def read_logs(handler):
        filename = getattr(handler, 'baseFilename', None)
        if not filename:
            return ''

        handler.flush()
        try:
            text = open(filename).read()
        except IOError:
            return ''

        name = os.path.basename(filename)
        header = '\n'.join(['logfile %s' % name, '-' * 50, ''])
        return '\n\n' + header + text.strip()

    logger = getattr(item, '_logger', None)
    handlers = getattr(logger, 'handlers', [])
    extras['logs'] = ''.join(read_logs(h) for h in handlers)
