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

    def test_contained_subgait(self):
        subgait = 'test'
        graph = SubgaitGraph({'start': {'to': subgait}, subgait: {'to': 'end'}})
        self.assertTrue(subgait in graph)

    def test_not_contained_subgait(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        self.assertFalse('test' in graph)

    def test_get_correct_to_transition(self):
        subgait = 'test'
        graph = SubgaitGraph({'start': {'to': subgait}, subgait: {'to': 'end'}})
        self.assertEqual(graph[(subgait, 'to')], 'end')

    def test_get_correct_stop_transition(self):
        subgait = 'test'
        graph = SubgaitGraph({'start': {'to': subgait}, subgait: {'stop': 'end'}})
        self.assertEqual(graph[(subgait, 'stop')], 'end')

    def test_get_no_subgait(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        with self.assertRaises(KeyError):
            graph[('2', 'to')]

    def test_get_invalid_transition(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        self.assertIsNone(graph[('1', 'stop')])
