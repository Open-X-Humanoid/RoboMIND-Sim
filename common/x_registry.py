from typing import Dict, Type, Optional, Tuple

class Registry:
    _registry: Dict[Tuple[str], Type[object]] = {}

    @classmethod
    def register(cls, name: str) -> callable:
        """Decorator for registering subclasses to unified registry"""
        def decorator(robot_cls: Type[object]) -> Type[object]:
            cls._registry[(name)] = robot_cls
            return robot_cls
        return decorator

    @classmethod
    def create(cls, name: str, *args, **kwargs) -> Optional[object]:
        """Create instance based on base class and subclass name"""
        robot_cls = cls._registry.get((name))
        if robot_cls is None:
            raise ValueError(f"Unknown type: {name}")
        return robot_cls(*args, **kwargs)