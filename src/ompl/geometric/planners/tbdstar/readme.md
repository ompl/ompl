# BIT*: Sampling-based optimal planning via batch informed trees

## Implementation Details

### Search Queue
In a nutshell: BIT* uses dual queues with lazy vertex expansion. 

#### Vertex Queue
The vertex queue is implemented as an `std::multimap`, which uses the lexographical key `[ g_t(v) + h^hat(v) ; g_t(v) ]` for sorting*. The underlying map is not used as a queue in the formal sense, i.e., we are not repeatedly pushing and popping elements. Rather, we keep track of the element at the conceptual front of the queue via an iterator into the map, which we refer to as the _token_. We "pop the element at the front" by returning it and moving the token to the next element. The big advantage of this approach is that we don't have to rebuild the entire queue after every search iteration - we can simply move the token to the first element in the map (`O(1)` vs at least `O(n)`). 

The rationale behind using `std::multimap` instead of any other associative container is that it stores its sorting key alongside the corresponding element (in contrast to the other viable option, `std::multiset`). This is important because we always know the keys the map is sorted by (even though the keys may be outdated because of an improved cost-to-come value (`g_t(v)`), and thus can easily check the sanity of the queue.

* Maybe with inflated heuristics, but that's not important for the sake of this discussion.