#ifndef USER_WHO_WANTS_TO_MOCK_AWAY_HPP_
#define USER_WHO_WANTS_TO_MOCK_AWAY_HPP_

#include <vector>
#include "mo.hpp"

struct pathfinder
{
    virtual std::vector<const base*> find_path(const base*, const base*) = 0;
};

struct dspfinder
{
    virtual const dsp* find_dsp(const base*, int id) = 0;
};

class you_can_unit_test_me_easily
{
public:
    you_can_unit_test_me_easily(pathfinder& pf, dspfinder& df);

    size_t foo(const base* start, const base* finish);
    bool bar(const base* start, int dsp_id);

private:
    pathfinder& pf_;
    dspfinder& df_;
};

#endif /* USER_WHO_WANTS_TO_MOCK_AWAY_HPP_ */
