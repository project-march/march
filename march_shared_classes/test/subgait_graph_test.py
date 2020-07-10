import unittest

from parameterized import parameterized

from march_shared_classes.exceptions.gait_exceptions import NonValidGaitContent
from march_shared_classes.gait.subgait_graph import SubgaitGraph


class SubgaitGraphTest(unittest.TestCase):

    @parameterized.expand([
        ('minimal', {'start': {'to': '1'}, '1': {'to': 'end'}}),
        ('stoppable loop', {'start': {'to': '1'}, '1': {'to': '2'}, '2': {'to': '1', 'stop': 'end'}}),
    ])
    def test_valid_graph(self, name, graph):
        SubgaitGraph(graph)

    @parameterized.expand([
        ('missing start', {'1': {'to': 'end'}}),
        ('nonexisting name', {'start': {'to': '2'}}),
        ('no transitions', {'start': {'invalid': 'yes'}}),
        ('equal transitions', {'start': {'to': '1', 'stop': '1'}, '1': {'to': 'end'}}),
        ('transition to start', {'start': {'to': '1'}, '1': {'to': 'start', 'stop': 'end'}}),
        ('no transitions to end', {'start': {'to': '1'}, '1': {'to': '2'}, '2': {'to': '1'}}),
        ('end not reachable from all',
         {'start': {'to': '1'}, '1': {'to': '2', 'stop': 'end'}, '2': {'to': '3'}, '3': {'to': '2'}}),
    ])
    def test_invalid_graph(self, name, graph):
        with self.assertRaises(NonValidGaitContent):
            SubgaitGraph(graph)
