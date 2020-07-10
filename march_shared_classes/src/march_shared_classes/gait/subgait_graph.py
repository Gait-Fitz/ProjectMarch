from collections import deque

from march_shared_classes.exceptions.gait_exceptions import NonValidGaitContent


class SubgaitGraph(object):
    START = 'start'
    END = 'end'

    def __init__(self, graph):
        self._graph = graph
        self.validate()

    def validate(self):
        """Validates the graph and raises an exception when not valid.

        This method checks a few things to prove consistency:
        1. Checks that a `start` and `end` state exist
        2. Checks that is possible to get to every state from `start`
        3. Checks that it is possible to get from every state to `end`
        4. Checks that all subgaits do not have equal `stop` and `to` transitions
        """
        if self.START not in self._graph:
            raise NonValidGaitContent(msg='There is no start state')

        queue = deque([(self.START, set())])
        visited = {}
        while len(queue) != 0:
            name, from_subgaits = queue.popleft()
            if name in visited:
                visited[name] = visited[name].union(from_subgaits)
                continue

            visited[name] = from_subgaits
            if name == self.END:
                continue

            self._validate_subgait(name)
            subgait = self._graph[name]
            if 'to' in subgait:
                from_subgaits = from_subgaits.copy()
                from_subgaits.add(name)
                queue.append((subgait['to'], from_subgaits))
            if 'stop' in subgait:
                from_subgaits = from_subgaits.copy()
                from_subgaits.add(name)
                queue.append((subgait['stop'], from_subgaits))

        self._validate_visited(visited)

    def _validate_subgait(self, name):
        subgait = self._graph.get(name)
        if subgait is None:
            raise NonValidGaitContent(msg='Subgait {n} is not a subgait in the graph'.format(n=name))
        if 'to' not in subgait and 'stop' not in subgait:
            raise NonValidGaitContent(msg='Subgait {n} has no transitions'.format(n=name))
        if subgait.get('to') == subgait.get('stop'):
            raise NonValidGaitContent(msg='`to` and `stop` transition cannot be the same')

    def _validate_visited(self, visited):
        if len(visited[self.START]) != 0:
            raise NonValidGaitContent(msg='Transition to start is not allowed')
        if self.END not in visited:
            raise NonValidGaitContent(msg='There are no transitions to `end`')
        if len(visited[self.END]) != len(self._graph):
            raise NonValidGaitContent(msg='`end` is not reachable from all subgaits')

    def __getitem__(self, item):
        pass

    def __iter__(self):
        pass
