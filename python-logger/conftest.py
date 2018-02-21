import os
import pytest
import logging
from process import process
from py.xml import html

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

@pytest.mark.optionalhook
def pytest_html_results_table_header(cells):
    cells.pop()
    cells.insert(-1, html.th('Start', class_='sortable time', col='time'))

@pytest.mark.optionalhook
def pytest_html_results_table_row(report, cells):
    cells.pop()
    cells.insert(-1, html.td('%.3f' % report._start, class_='col-time'))

@pytest.mark.optionalhook
def pytest_html_results_table_html(report, data):
    add_html_logs(report, data)

@pytest.mark.hookwrapper
def pytest_runtest_makereport(item, call):
    outcome = yield
    add_html_extras(item, call, outcome)

def add_html_logs(report, data):
    for handler in report._handlers:
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
    html = getattr(item.config, '_html')
    if html:
        report = output.get_result()

        if not hasattr(html, '_start'):
            html._start = call.start
        report._start = call.start - html._start

        logger = getattr(item, '_logger', None)
        handlers = getattr(logger, 'handlers', [])
        report._handlers = handlers

setuplgr = logging.getLogger('setup')
setuplgr.addHandler(logging.NullHandler())

daemonlgr = logging.getLogger('daemon')
daemonlgr.addHandler(logging.NullHandler())

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
