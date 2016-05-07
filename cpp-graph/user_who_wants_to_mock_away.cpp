#include "user_who_wants_to_mock_away.hpp"

you_can_unit_test_me_easily::you_can_unit_test_me_easily(pathfinder& pf, dspfinder& df)
    : pf_(pf), df_(df)
{}

size_t you_can_unit_test_me_easily::foo(const base* start, const base* finish)
{
    return pf_.find_path(start, finish).size();
}

bool you_can_unit_test_me_easily::bar(const base* start, int dsp_id)
{
    return df_.find_dsp(start, dsp_id);
}
