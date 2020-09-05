import unittest

from parameterized import parameterized

from march_shared_classes.exceptions.gait_exceptions import SubgaitGraphError
from march_shared_classes.gait.subgait_graph import SubgaitGraph


class SubgaitGraphTest(unittest.TestCase):

    @parameterized.expand([
        ('minimal', {'start': {'to': '1'}, '1': {'to': 'end'}}),
        ('stoppable loop', {'start': {'to': '1'}, '1': {'to': '2'}, '2': {'to': '1', 'stop': 'end'}}),
        ('increase decrease graph',
         {'start': {'to': '1'}, '1': {'to': 'end', 'increase_size': '2', 'decrease_size': '3'}, '2': {'to': 'end'},
          '3': {'to': 'end'}}),
    ])
    def test_valid_graph(self, name, graph):
        self.assertIsInstance(SubgaitGraph(graph), SubgaitGraph)

    @parameterized.expand([
        ('missing start', {'1': {'to': 'end'}}),
        ('nonexisting name', {'start': {'to': '2'}}),
        ('no transitions', {'start': {}}),
        ('no to transition', {'start': {'to': '1'}, '1': {'stop': 'end'}}),
        ('no subgaits', {}),
        ('invalid transitions', {'start': {'to': '1'}, '1': {'to': 'end', 'from': 'start'}}),
        ('equal transitions', {'start': {'to': '1', 'stop': '1'}, '1': {'to': 'end'}}),
        ('equal invalid transition',
         {'start': {'to': '1'}, '1': {'to': '2', 'loop': '2'}, '2': {'to': '1', 'stop': 'end'}}),
        ('transition to start', {'start': {'to': '1'}, '1': {'to': 'start', 'stop': 'end'}}),
        ('no transitions to end', {'start': {'to': '1'}, '1': {'to': '2'}, '2': {'to': '1'}}),
        ('end not reachable from all',
         {'start': {'to': '1'}, '1': {'to': '2', 'stop': 'end'}, '2': {'to': '3'}, '3': {'to': '2'}}),
        ('not all reachable from start', {'start': {'to': '1'}, '1': {'to': 'end'}, '2': {'to': 'end'}}),
    ])
    def test_invalid_graph(self, name, graph):
        with self.assertRaises(SubgaitGraphError):
            SubgaitGraph(graph)

    def test_stoppable_graph(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': '2'}, '2': {'to': '1', 'stop': 'end'}})
        self.assertTrue(graph.is_stoppable())

    def test_not_stoppable_graph(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        self.assertFalse(graph.is_stoppable())

    def test_subgaits_from_start(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        self.assertListEqual(graph.start_subgaits(), ['1'])

    def test_subgait_transition_to_end(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        self.assertListEqual(graph.end_subgaits(), ['1'])

    def test_subgait_transitions_to_end(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end', 'stop': '2'}, '2': {'to': 'end'}})
        self.assertListEqual(graph.end_subgaits(), ['1', '2'])

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
        stop_subgait = 'stopping'
        graph = SubgaitGraph(
            {'start': {'to': subgait}, subgait: {'to': 'end', 'stop': stop_subgait}, stop_subgait: {'to': 'end'}})
        self.assertEqual(graph[(subgait, 'stop')], stop_subgait)

    def test_get_no_subgait(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        with self.assertRaises(KeyError):
            graph.__getitem__(('2', 'to'))

    def test_get_invalid_transition(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        self.assertIsNone(graph[('1', 'stop')])

    def test_iter_transitions(self):
        graph = SubgaitGraph({'start': {'to': '1'}, '1': {'to': '2'}, '2': {'to': '1', 'stop': 'end'}})
        self.assertListEqual(list(iter(graph)), [('1', '2'), ('start', '1'), ('2', '1'), ('2', 'end')])

    def test_iter_transitions_without_subgaits(self):
        graph = SubgaitGraph({'start': {'to': 'end'}})
        self.assertListEqual(list(iter(graph)), [('start', 'end')])

    def test_iter_transitions_with_increase_decrease(self):
        graph = SubgaitGraph(
            {'start': {'to': '1'}, '1': {'to': 'end', 'increase_size': '2', 'decrease_size': '3'}, '2': {'to': 'end'},
             '3': {'to': 'end'}})
        self.assertListEqual(list(iter(graph)), [('1', 'end'), ('start', '1'), ('3', 'end'), ('2', 'end')])

    def test_equal_graphs(self):
        graph1 = SubgaitGraph({'start': {'to': 'end'}})
        graph2 = SubgaitGraph({'start': {'to': 'end'}})
        self.assertTrue(graph1 == graph2)

    def test_not_equal_graphs(self):
        graph1 = SubgaitGraph({'start': {'to': '1'}, '1': {'to': 'end'}})
        graph2 = SubgaitGraph({'start': {'to': 'end'}})
        self.assertFalse(graph1 == graph2)
