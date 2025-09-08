import networkx as nx
import matplotlib.pyplot as plt
from dataclasses import dataclass

@dataclass
class Node:
    edge: str
    val: float
    valid: bool = True


class GPLot:
    def __init__(self):
        self.graph = nx.DiGraph(directed=True)
        self.val_map: dict[str, bool] = dict()
        
    def add_edge(self, node_start: Node, node_end: Node):
        self.graph.add_edge(node_start.edge, node_end.edge)
        self.val_map.update({node_start.edge: node_start.val, node_end.edge: node_end.val})

    def plot(self):
        values = [self.val_map.get(node, 0.25) for node in self.graph.nodes()]
        red_edges = [(edge1, edge2) for edge1, edge2 in zip(list(self.graph.nodes())[:-1], list(self.graph.nodes())[1:]) if self.val_map[edge1] < 0.5 and self.val_map[edge2] < 0.5 and self.graph.has_edge(edge1, edge2)]
        black_edges = [edge for edge in self.graph.edges() if edge not in red_edges]
        pos = nx.spring_layout(self.graph)
        nx.draw_networkx_nodes(self.graph, pos, cmap=plt.get_cmap('prism'), 
                            node_color = values, node_size = 300)
        nx.draw_networkx_labels(self.graph, pos)
        nx.draw_networkx_edges(self.graph, pos, edgelist=red_edges, edge_color='r', arrows=True)
        nx.draw_networkx_edges(self.graph, pos, edgelist=black_edges, arrows=True)
        plt.show()


if __name__ == "__main__":
    g = GPLot()
    tile1 = Node(edge="Tile1", val=1)
    map1 = Node(edge="Map1", val=0)
    map2 = Node(edge="Map2", val=1)
    train1 = Node(edge="Train1", val=0)
    train2 = Node(edge="Train2", val=0)
    train3 = Node(edge="Train3", val=1)
    g.add_edge(tile1, map1)
    g.add_edge(tile1, map2)
    g.add_edge(map1, train1)
    g.add_edge(map1, train2)
    g.add_edge(map2, train3)
    print(g.graph.has_edge("Train2", 'Train3'))
    g.plot()
        