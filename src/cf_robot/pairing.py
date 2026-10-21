"""Human-readable, per-run codes for the loopback controller bridge."""

import secrets


ADJECTIVES = tuple("""
able agile amber ample ancient arctic autumn azure
calm candid careful cedar cheerful clean clear clever
cloudy cobalt cool coral cosmic cozy crisp curious
daring dawn deep desert eager early earthy easy
even fair famous fancy fast festive fine floral
foggy forest fresh friendly frosty gentle giant glad
golden grand grassy great green happy hardy hazy
helpful hidden honest humble icy indigo ivory jade
jolly joyful keen kind leafy lemon light lively
local lucky lunar magic mellow merry mild misty
modern mossy neat nimble noble northern ocean olive
orange patient peach peaceful pearly pine plain playful
polar proud pure quick quiet rapid ready royal
ruby rustic sandy shiny silent silver simple snowy
solar solid spring steady still stone strong sunny
swift tender tidy tiny trusty vivid warm wild
""".split())

NOUNS = tuple("""
acorn anchor apple apricot arrow badger bamboo basket
beacon bear beaver beetle berry birch bison blossom
boat breeze brook buffalo cabin cactus camel candle
canyon carrot castle cedar cherry cloud clover coast
deer delta desert dolphin dove dragon dragonfly duck
eagle elm falcon feather fern field finch firefly
fjord flame flower forest fossil fox frog garden
gazelle gecko ginger glacier grove harbor hare hawk
hazel heron hill honey island ivy jaguar jasper
jay juniper kite kitten koala lake lark laurel
leaf lemon leopard lily lion lotus lynx maple
marble meadow melon mint moose moss mountain ocean
olive orchid otter owl panda panther parrot peach
pearl pebble pelican penguin pepper petal pine planet
plum pond poppy puffin pumpkin rabbit raven reef
river robin rocket rose sage salmon seal sparrow
""".split())


def generate_pairing_code():
    return f"{secrets.choice(ADJECTIVES)}-{secrets.choice(NOUNS)}"
