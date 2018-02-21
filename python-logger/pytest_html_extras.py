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
    add_html_logs(report, data)

@pytest.mark.hookwrapper
def pytest_runtest_makereport(item, call):
    outcome = yield
    add_html_extras(item, call, outcome)

def add_html_logs(report, data):
    for handler in report._extras['handlers']:
        filename = getattr(handler, 'baseFilename', None)
        if filename:
            handler.flush()
            try:
                text = open(filename).read()
            except IOError:
                continue

            name = os.path.basename(filename)
            header = '\n'.join(['logfile %s' % name, '-' * 50, ''])
            data[0].append(('\n\n' + header + text.strip()))

def add_html_extras(item, call, output):
    try:
        html = item.config._html
    except AttributeError:
        return

    report = output.get_result()
    report._extras = extras = {}

    if not hasattr(html, '_start'):
        html._start = call.start
    extras['start'] = call.start - html._start

    extras['markers'] = sorted(filter(item.get_marker, item.keywords.keys()))

    try:
        extras['handlers'] = item._logger.handlers
    except AttributeError:
        extras['handlers'] = []
