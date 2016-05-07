#ifndef GRAPH_HPP_
#define GRAPH_HPP_

#include <algorithm>
#include <deque>
#include <vector>
#include <queue>
#include "mo.hpp"

class graph
{
public:
    template <typename Dev>
    void add_device(const Dev* dev);
    void add_connector(const connector*);

    bool link(const hwlink* lnk);
    bool link(const cablink*);

    template <typename Pred>
    const base* find(const base* start, Pred pred);

    template <typename Mo, typename Pred>
    const Mo* find(const base* start, Pred pred);

    template <typename Pred>
    std::vector<const base*> find_path(const base* start, Pred pred);

    template <typename Mo, typename Pred>
    std::vector<const base*> find_path(const base* start, Pred pred);

public:
    struct node
    {
        node() {}
        node(const base* m): mo(m) {}

        const base* mo = {};
        std::vector<node*> neighbors;
        node* previous = {};
    };

    class cache
    {
    public:
        explicit cache(std::deque<node>& nodes): nodes_(nodes) {}

        node* find(const base* mo);
        void refresh();
        void dirty();

    private:
        std::deque<node>& nodes_;
        std::vector<node*> mo_sorted;
        bool is_dirty = false;
    };

    struct cleaner;
    struct finder;

    std::deque<node> nodes_;
    cache cache_ {nodes_};
};

inline graph::node* graph::cache::find(const base* mo)
{
    refresh();
    auto cmp_mo = [](node* lhs, const base* mo){ return lhs->mo < mo; };
    auto found = std::lower_bound(mo_sorted.begin(), mo_sorted.end(), mo, cmp_mo);
    if (found == mo_sorted.end())
    {
        return nullptr;
    }
    node* nfound = *found;
    if (nfound->mo == mo)
    {
        return nfound;
    }
    return nullptr;
}

inline void graph::cache::refresh()
{
    if (is_dirty)
    {
        mo_sorted.clear();
        for (node& nd : nodes_)
        {
            mo_sorted.push_back(&nd);
        }
        auto mo_cmp = [](node* lhs, node* rhs){ return lhs->mo < rhs->mo; };
        std::sort(mo_sorted.begin(), mo_sorted.end(), mo_cmp);
        is_dirty = false;
    }
}

inline void graph::cache::dirty()
{
    is_dirty = true;
}

template <typename Dev>
inline void graph::add_device(const Dev* dev)
{
    nodes_.push_back({dev});
    node& devnode = nodes_.back();
    for (const hwport* port : dev->ports)
    {
        nodes_.push_back({port});
        node& portnode = nodes_.back();
        devnode.neighbors.push_back(&portnode);
        portnode.neighbors.push_back(&devnode);
    }
    cache_.dirty();
}

inline void graph::add_connector(const connector* con)
{
    nodes_.push_back({con});
    cache_.dirty();
}

inline bool graph::link(const hwlink* lnk)
{
    if (node* nsrc = cache_.find(lnk->src))
    {
        if (node* ndst = cache_.find(lnk->dst))
        {
            nsrc->neighbors.push_back(ndst);
            ndst->neighbors.push_back(nsrc);
            return true;
        }
    }
    return false;
}

inline bool graph::link(const cablink* lnk)
{
    if (node* nsrc = cache_.find(lnk->src))
    {
        if (node* ndst = cache_.find(lnk->dst))
        {
            nsrc->neighbors.push_back(ndst);
            ndst->neighbors.push_back(nsrc);
            return true;
        }
    }
    return false;
}

struct graph::cleaner
{
    ~cleaner()
    {
        for (node* nd : nodes)
        {
            nd->previous = nullptr;
        }
    }

    void add(node* nd)
    {
        nodes.push_back(nd);
    }

    bool operator()(node* nd, node* previous)
    {
        if (nd->previous)
        {
            return true;
        }

        nd->previous = previous;
        add(nd);
        return false;
    }

    std::vector<node*> nodes;
};

struct graph::finder
{
    explicit finder(cache& cac): cach(cac) {}

    template <typename Mo, typename Pred>
    node* find(const base* start, Pred pred)
    {
        if (node* nstart = cach.find(start))
        {
            return find<Mo>(nstart, pred);
        }
        return nullptr;
    }

    template <typename Mo, typename Pred>
    node* find(node* start, Pred pred)
    {
        visited(start, &nullnode);
        frontier.push(start);

        while (!frontier.empty())
        {
            for (node* neighbor : frontier.front()->neighbors)
            {
                if (visited(neighbor, frontier.front()))
                {
                    continue;
                }
                if (const Mo* mo = dynamic_cast<const Mo*>(neighbor->mo))
                {
                    if (pred(*mo))
                    {
                        return neighbor;
                    }
                }
                frontier.push(neighbor);
            }
            frontier.pop();
        }
        return nullptr;
    }

    cache& cach;
    node nullnode;
    cleaner visited;
    std::queue<node*> frontier;
};

template <typename Pred>
inline const base* graph::find(const base* start, Pred pred)
{
    return find<base>(start, pred);
}

template <typename Mo, typename Pred>
inline const Mo* graph::find(const base* start, Pred pred)
{
    finder fdr(cache_);
    if (node* found = fdr.find<Mo>(start, pred))
    {
        return dynamic_cast<const Mo*>(found->mo);
    }
    return nullptr;
}

template <typename Pred>
inline std::vector<const base*> graph::find_path(const base* start, Pred pred)
{
    return find_path<base>(start, pred);
}

template <typename Mo, typename Pred>
inline std::vector<const base*> graph::find_path(const base* start, Pred pred)
{
    finder fdr(cache_);
    if (node* found = fdr.find<Mo>(start, pred))
    {
        std::vector<const base*> path;
        while (found->previous)
        {
            path.push_back(found->mo);
            found = found->previous;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
    return {};
}

#endif /* GRAPH_HPP_ */
