extends Node
class_name Require

# Safe helper to fetch a node.
# If the path is empty or the node is missing, logs an error and returns null.
# Type is *not* enforced anymore; expected_type is only used for messaging.
static func node(host: Node, path: NodePath, expected_type: String) -> Node:
	if path == NodePath(""):
		push_error("%s: Node path is empty (expected '%s')" % [host.name, expected_type])
		return null

	var n: Node = host.get_node_or_null(path)
	if n == null:
		push_error("%s: Node '%s' not found (expected '%s')" % [host.name, String(path), expected_type])
		return null

	# We no longer hard-check n.get_class() against expected_type, to avoid
	# false errors with subclasses (ColorRect vs Control, custom classes, etc.).
	return n
