"""
Causal reasoning: DAG causa→efecto, intervenciones (do-calculus), contrafácticos.
"""
from __future__ import annotations

from typing import Any, Dict, List, Set, Tuple

try:
    import networkx as nx
except ImportError:
    nx = None


class CausalGraph:
    """
    Grafo acíclico dirigido de relaciones causales.
    Ejemplo: Rain → Wet Ground, Sprinkler → Wet Ground, Wet Ground → Slippery.
    """

    def __init__(self) -> None:
        if nx is None:
            raise RuntimeError("networkx required for CausalGraph")
        self.graph = nx.DiGraph()
        self.observations: Dict[str, float] = {}

    def add_variable(self, name: str) -> None:
        self.graph.add_node(name)

    def add_causal_edge(self, cause: str, effect: str, strength: float = 1.0) -> None:
        self.graph.add_edge(cause, effect, strength=strength)

    def observe(self, variable: str, value: float) -> None:
        self.observations[variable] = value

    def get_causes(self, effect: str) -> List[str]:
        return list(self.graph.predecessors(effect))

    def get_effects(self, cause: str) -> List[str]:
        return list(self.graph.successors(cause))

    def find_path(self, cause: str, effect: str) -> List[str]:
        try:
            return nx.shortest_path(self.graph, cause, effect)
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            return []

    def do_intervention(self, variable: str, value: float) -> Dict[str, Any]:
        """Intervención do(X=x): rompe aristas entrantes (do-calculus de Pearl)."""
        original_edges = list(self.graph.in_edges(variable))
        for source, _ in original_edges:
            self.graph.remove_edge(source, variable)
        self.observations[variable] = value
        affected = self._propagate_effects(variable)
        for source, target in original_edges:
            self.graph.add_edge(source, target)
        return {
            "intervention": {variable: value},
            "affected_variables": list(affected),
            "causal_path": self._trace_causal_paths(variable),
        }

    def counterfactual(
        self, variable: str, value: float, reality: Dict[str, float]
    ) -> Dict[str, Any]:
        """Contrafáctico: qué pasaría si variable hubiera sido value dado reality."""
        original_observations = self.observations.copy()
        self.observations = dict(reality)
        counterfactual_result = self.do_intervention(variable, value)
        differences: Dict[str, Any] = {}
        for var in counterfactual_result["affected_variables"]:
            if var in reality:
                cf_val = self.observations.get(var)
                differences[var] = {
                    "reality": reality[var],
                    "counterfactual": cf_val,
                    "change": (cf_val or 0) - reality.get(var, 0),
                }
        self.observations = original_observations
        return {
            "counterfactual_scenario": {variable: value},
            "reality": reality,
            "differences": differences,
            "causal_explanation": self._explain_changes(differences),
        }

    def _propagate_effects(self, source: str) -> Set[str]:
        affected: Set[str] = set()
        queue = [source]
        visited = {source}
        while queue:
            current = queue.pop(0)
            for effect in self.get_effects(current):
                if effect not in visited:
                    visited.add(effect)
                    affected.add(effect)
                    queue.append(effect)
                    if current in self.observations:
                        edge_data = self.graph.get_edge_data(current, effect)
                        strength = (
                            edge_data.get("strength", 1.0) if edge_data else 1.0
                        )
                        self.observations[effect] = (
                            self.observations[current] * strength
                        )
        return affected

    def _trace_causal_paths(self, source: str) -> Dict[str, List[str]]:
        paths: Dict[str, List[str]] = {}
        for node in self.graph.nodes():
            if node != source:
                path = self.find_path(source, node)
                if path:
                    paths[node] = path
        return paths

    def _explain_changes(self, differences: Dict[str, Any]) -> str:
        if not differences:
            return "No observable changes"
        explanations = []
        for var, change_info in differences.items():
            change = change_info.get("change", 0)
            direction = "increased" if change > 0 else "decreased"
            magnitude = abs(change)
            explanations.append(
                f"{var} {direction} by {magnitude:.2f} "
                f"(from {change_info.get('reality', 0):.2f} to {change_info.get('counterfactual', 0):.2f})"
            )
        return "; ".join(explanations)

    def visualize(self) -> Dict[str, Any]:
        return {
            "nodes": list(self.graph.nodes()),
            "edges": [
                {"source": u, "target": v, "strength": data.get("strength", 1.0)}
                for u, v, data in self.graph.edges(data=True)
            ],
            "observations": self.observations,
        }


class CausalReasoner:
    """Interfaz de razonamiento causal por dominios."""

    def __init__(self) -> None:
        self.graphs: Dict[str, CausalGraph] = {}

    def create_domain(self, domain_name: str, structure: Dict[str, Any]) -> CausalGraph:
        """
        structure: { 'variables': [...], 'edges': [(cause, effect, strength), ...] }
        """
        graph = CausalGraph()
        for var in structure.get("variables", []):
            graph.add_variable(var)
        for edge in structure.get("edges", []):
            if len(edge) >= 3:
                graph.add_causal_edge(edge[0], edge[1], float(edge[2]))
            elif len(edge) == 2:
                graph.add_causal_edge(edge[0], edge[1])
        self.graphs[domain_name] = graph
        return graph

    def reason_about_action(
        self, domain: str, action: str, current_state: Dict[str, float]
    ) -> Dict[str, Any]:
        """Consecuencias de una acción (acción como intervención)."""
        if domain not in self.graphs:
            return {"error": f"Unknown domain: {domain}"}
        graph = self.graphs[domain]
        for var, value in current_state.items():
            graph.observe(var, value)
        parts = action.split("_to_")
        if len(parts) == 2:
            variable = parts[0].replace("set_", "")
            try:
                value = float(parts[1])
            except ValueError:
                return {"error": "Invalid action value"}
            result = graph.do_intervention(variable, value)
            return {
                "action": action,
                "predicted_effects": result["affected_variables"],
                "causal_paths": result["causal_path"],
                "explanation": f"Setting {variable} to {value} will affect: {', '.join(result['affected_variables'])}",
            }
        return {"error": "Invalid action format (use set_X_to_Y)"}

    def explain_why(
        self, domain: str, effect: str, potential_causes: List[str]
    ) -> str:
        if domain not in self.graphs:
            return f"Unknown domain: {domain}"
        graph = self.graphs[domain]
        actual_causes: List[Tuple[str, List[str]]] = []
        for cause in potential_causes:
            path = graph.find_path(cause, effect)
            if path:
                actual_causes.append((cause, path))
        if not actual_causes:
            return f"None of {potential_causes} cause {effect}"
        explanations = [
            f"{cause} causes {effect} via: {' → '.join(path)}"
            for cause, path in actual_causes
        ]
        return "; ".join(explanations)
