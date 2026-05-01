from typing import Optional



TASKS = {}  # name -> {"fn": callable, "label": str, "aliases": list}

def task(label: str, aliases: Optional[list[str]] = None):
	"""
	Decorator to register a task function with its help text label.
	
	Args:
		label: Help text describing what the task does
		aliases: Optional list of alternative names for this task
	
	Example:
		@task("Build the ROS workspace", aliases=["b"])
		def build(packages_select: Optional[str] = None):
			...
	"""
	def decorator(fn):
		# Extract the task name from the function name (remove 'target_' prefix if present)
		# and convert underscores to hyphens for consistency with CLI conventions
		name = fn.__name__.replace("target_", "").replace("_", "-")
		TASKS[name] = {"fn": fn, "label": label, "aliases": aliases or []}
		# Also register aliases
		for alias in (aliases or []):
			TASKS[alias] = {"fn": fn, "label": label, "aliases": []}
		return fn
	return decorator


