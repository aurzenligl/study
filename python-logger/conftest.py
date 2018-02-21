import os
import pytest
import logging
from process import process

from datetime import datetime
from py.xml import html
import pytest

@pytest.mark.optionalhook
def pytest_html_results_table_header(cells):
    cells.insert(-1, html.th('Start', class_='sortable time', col='time'))
    cells.append(html.th('Logs'))

@pytest.mark.optionalhook
def pytest_html_results_table_row(report, cells):
    cells.insert(-1, html.td('%.3f' % report._start, class_='col-time'))
    cells.append(html.td('No loggies captured.'))

@pytest.mark.optionalhook
def pytest_html_results_table_html(report, data):
    for handler in report._handlers:
        filename = getattr(handler, 'baseFilename', None)
        if filename:
            handler.flush()
            try:
                text = open(filename).read()
            except IOError:
                continue
            data[0].append(html.div(text.strip()))

@pytest.mark.hookwrapper
def pytest_runtest_makereport(item, call):
    pytest_html = item.config.pluginmanager.getplugin('html')

    if not hasattr(pytest_html, '_start'):
        pytest_html._start = call.start

    outcome = yield
    report = outcome.get_result()
    extra = getattr(report, 'extra', [])
    report._start = call.start - pytest_html._start
    report._handlers = item._logger.handlers
    if report.when == 'call':
        # always add url to report
        extra.append(pytest_html.extras.url('http://www.google.com/'))
        extra.append(pytest_html.extras.json({'name': 'pytest'}))
        extra.append(pytest_html.extras.text('Add some simple Text'))
        extra.append(pytest_html.extras.text('some string', name='Different title'))
        report.extra = extra

def pytest_addoption(parser):
    parser.addoption('--count', type='int', metavar='COUNT',
                     help='Run each test the specified number of times')
    parser.addoption('--error', action='store_true',
                     help='Cause failure during test run')

def pytest_logger_logdirlink(config):
    return os.path.join(os.path.dirname(__file__), 'logs')

def pytest_logger_config(logger_config):
    logger_config.add_loggers(['setup', 'im', 'data', 'proc', 'daemon', 'thelib'], stdout_level=logging.INFO)
    logger_config.set_log_option_default('thelib')

def pytest_generate_tests(metafunc):
    count = metafunc.config.option.count
    if count is not None:
        for i in range(count):
            metafunc.addcall()

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

daemonlgr = logging.getLogger('daemon')
daemonlgr.addHandler(logging.NullHandler())

logging.root.setLevel(logging.NOTSET)

@pytest.yield_fixture(scope='session', autouse=True)
def sessionfixture():
    setuplgr.info('session begins')
    yield
    setuplgr.info('session ends')

@pytest.yield_fixture
def the_error_fixture(request):
    setuplgr.info('before...')
    yield request.config.getoption('error')
    setuplgr.info('...after')

@pytest.yield_fixture(scope='session')
def daemon(request):
    with process('./printdaemon', stdoutlgr=daemonlgr):
        yield
