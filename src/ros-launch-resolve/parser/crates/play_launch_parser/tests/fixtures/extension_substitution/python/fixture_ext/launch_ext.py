"""A launch extension for play_launch's tests (#0099).

Substitutions exposed in each way `launch.frontend.expose_substitution`
accepts, written the way `ceilbot_profiles.launch_ext` writes its own
(`$(robot-value ...)`). Registered under the entry point group
`launch.frontend.launch_extension` by `fixture_ext-0.0.0.dist-info`.
"""
from launch.frontend import expose_substitution
from launch.substitution import Substitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions


@expose_substitution('fixture-join')
class Join(Substitution):
    """$(fixture-join A B ...): the arguments joined with '_'."""

    def __init__(self, parts):
        super().__init__()
        self.parts = [normalize_to_list_of_substitutions(p) for p in parts]

    @classmethod
    def parse(cls, data):
        return cls, {'parts': data}

    def perform(self, context):
        return '_'.join(perform_substitutions(context, p) for p in self.parts)


@expose_substitution('fixture-config')
class Config(Substitution):
    """$(fixture-config NAME): the launch configuration NAME, read from the context."""

    def __init__(self, name):
        super().__init__()
        self.name = normalize_to_list_of_substitutions(name)

    @classmethod
    def parse(cls, data):
        if len(data) != 1:
            raise TypeError(f'$(fixture-config) takes one argument, got {len(data)}')
        return cls, {'name': data[0]}

    def perform(self, context):
        return context.launch_configurations[perform_substitutions(context, self.name)]


class Upper(Substitution):
    """$(fixture-upper TEXT): TEXT in capitals."""

    def __init__(self, text):
        super().__init__()
        self.text = normalize_to_list_of_substitutions(text)

    def perform(self, context):
        return perform_substitutions(context, self.text).upper()


@expose_substitution('fixture-upper')
def parse_upper(data):
    """A parse function exposed by itself, which launch accepts too."""
    return Upper, {'text': data[0]}
