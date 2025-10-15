# the parent of this node. If a node's parent is its own index,
# then it is a root.
uint32 parent

# for the root of a connected component, the number of components
# connected to it. For intermediate values, it's not meaningful.
uint32 size
