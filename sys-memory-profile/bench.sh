#!/bin/bash

valgrind --tool=massif --massif-out-file=massif.out --threshold=0.01 \
--ignore-fn='hun()' \
--ignore-fn='std::string::_Rep::_S_create(unsigned long, unsigned long, std::allocator<char> const&)' \
--ignore-fn='std::__detail::_Hashtable_alloc<std::allocator<std::__detail::_Hash_node<std::pair<std::string const, std::vector<std::string, std::allocator<std::string> > >, true> > >::_M_allocate_buckets(unsigned long)' \
./a.out

massif-visualizer massif.out
