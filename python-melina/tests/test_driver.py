import os
import melina
import pytest

datadir = os.path.abspath(__file__ + '/../data')

def fromdata(basename):
    return os.path.join(datadir, basename)

class TestDriver():

    def test_options_fail_noinput(self, capsys):
        ret = melina.main('')
        assert ret == melina.EXIT_FAILURE
        assert capsys.readouterr()[1] == 'melina: error: missing input\n'

    def test_options_fail_unknownopt(self, capsys):
        ret = melina.main('--unknown')
        assert ret == melina.EXIT_FAILURE
        assert capsys.readouterr()[1] == 'melina: error: unrecognized arguments: --unknown\n'

    @pytest.mark.parametrize('args', [
        fromdata('short.meta'),
        fromdata('short.xml'),
        '--meta ' + fromdata('short.meta.concealed'),
        '--xml ' + fromdata('short.xml.concealed'),
    ])
    def test_input(self, capsys, args):
        ret = melina.main(args)
        assert ret == melina.EXIT_OK
        assert capsys.readouterr()[1] == 'Your input is beautiful! No output selected though.\n'

    @pytest.mark.parametrize('args', [
        fromdata('short.meta.concealed'),
        fromdata('short.xml.concealed'),
    ])
    def test_input_fail_parser_choice(self, capsys, args):
        ret = melina.main(args)
        assert ret == melina.EXIT_FAILURE
        assert 'Input type was not given and cannot be deduced' in capsys.readouterr()[1]

    def test_input_fail_multiple_choice(self, capsys):
        ret = melina.main('--meta --xml short.xml')
        assert ret == melina.EXIT_FAILURE
        assert '--xml: not allowed with argument --meta' in capsys.readouterr()[1]

    @pytest.mark.parametrize('args', [
        fromdata('wrong.meta'),
        fromdata('wrong.xml'),
    ])
    def test_input_fail_parsing(self, capsys, args):
        ret = melina.main(args)
        assert ret == melina.EXIT_FAILURE
        assert capsys.readouterr()[1] != ''

    def test_output(self, capsys, tmpdir):
        ret = melina.main('--meta-out %s --xml-out %s %s' % (tmpdir, tmpdir, fromdata('short.meta')))
        assert ret == melina.EXIT_OK
        assert capsys.readouterr()[1] == ''
        ret = melina.main(str(tmpdir.join('short.meta')))
        assert ret == melina.EXIT_OK
        ret = melina.main(str(tmpdir.join('short.xml')))
        assert ret == melina.EXIT_OK

    def test_output_stdout_meta(self, capsys, tmpdir):
        ret = melina.main('--meta-stdout --meta-out %s %s' % (tmpdir, fromdata('short.meta')))
        assert ret == melina.EXIT_OK
        assert capsys.readouterr()[0] == tmpdir.join('short.meta').read()

    def test_output_stdout_xml(self, capsys, tmpdir):
        ret = melina.main('--xml-stdout --xml-out %s %s' % (tmpdir, fromdata('short.xml')))
        assert ret == melina.EXIT_OK
        assert capsys.readouterr()[0] == tmpdir.join('short.xml').read()

    def test_output_fail_multiple_stdout(self, capsys):
        ret = melina.main('--meta-stdout --xml-stdout short.xml')
        assert ret == melina.EXIT_FAILURE
        assert '--xml-stdout: not allowed with argument --meta-stdout' in capsys.readouterr()[1]

    def test_demo(self, capsys):
        ret = melina.main(fromdata('demo.meta'))
        assert ret == melina.EXIT_OK
        assert 'Your input is beautiful! No output selected though.\n' in capsys.readouterr()[1]
