import "package:flutter/material.dart";
import "../../shared/models/entry_model.dart";
import "../../shared/services/rauli_repo.dart";
import "../../shared/widgets/quick_entry_dialog.dart";

class DashboardPage extends StatefulWidget {
  const DashboardPage({super.key});

  @override
  State<DashboardPage> createState() => _DashboardPageState();
}

class _DashboardPageState extends State<DashboardPage> {
  double _salesToday = 0.0;
  double _prodToday = 0.0;
  List<EntryModel> _last = const [];

  @override
  void initState() {
    super.initState();
    _refresh();
  }

  Future<void> _refresh() async {
    final s = await RauliRepo.totalSalesToday();
    final p = await RauliRepo.totalProductionToday();
    final last = await RauliRepo.lastEntries(limit: 20);
    if (!mounted) return;
    setState(() {
      _salesToday = s;
      _prodToday = p;
      _last = last;
    });
  }

  Future<void> _addSale() async {
    final res = await showDialog(
      context: context,
      builder: (_) => const QuickEntryDialog(
        title: "Registrar venta",
        amountLabel: "Monto (USD)",
        noteLabel: "Nota (ej: pan sobao x2)",
      ),
    );
    if (res is Map) {
      final a = (res["amount"] as num).toDouble();
      final n = (res["note"] ?? "").toString();
      await RauliRepo.addSale(amount: a, note: n);
      await _refresh();
    }
  }

  Future<void> _addProduction() async {
    final res = await showDialog(
      context: context,
      builder: (_) => const QuickEntryDialog(
        title: "Registrar produccion",
        amountLabel: "Cantidad (o costo)",
        noteLabel: "Nota (ej: galletas x30)",
      ),
    );
    if (res is Map) {
      final a = (res["amount"] as num).toDouble();
      final n = (res["note"] ?? "").toString();
      await RauliRepo.addProduction(amount: a, note: n);
      await _refresh();
    }
  }

  String _typeLabel(String t) => (t == "sale") ? "VENTA" : "PROD";

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text("RAULI"),
        actions: [
          IconButton(
            onPressed: _refresh,
            icon: const Icon(Icons.refresh),
            tooltip: "Actualizar",
          )
        ],
      ),
      body: Padding(
        padding: const EdgeInsets.all(16),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const Text(
              "Dashboard",
              style: TextStyle(fontSize: 22, fontWeight: FontWeight.bold),
            ),
            const SizedBox(height: 12),

            Wrap(
              spacing: 12,
              runSpacing: 12,
              children: [
                _KpiCard(title: "Ventas hoy", value: _salesToday.toStringAsFixed(2)),
                _KpiCard(title: "Produccion hoy", value: _prodToday.toStringAsFixed(2)),
                _KpiCard(title: "Movimientos", value: _last.length.toString()),
              ],
            ),

            const SizedBox(height: 16),

            Row(
              children: [
                Expanded(
                  child: FilledButton.icon(
                    onPressed: _addSale,
                    icon: const Icon(Icons.point_of_sale),
                    label: const Text(" + Venta"),
                  ),
                ),
                const SizedBox(width: 12),
                Expanded(
                  child: FilledButton.icon(
                    onPressed: _addProduction,
                    icon: const Icon(Icons.factory),
                    label: const Text(" + Produccion"),
                  ),
                ),
              ],
            ),

            const SizedBox(height: 16),
            const Text("Ultimos movimientos", style: TextStyle(fontWeight: FontWeight.w600)),
            const SizedBox(height: 8),

            Expanded(
              child: ListView.separated(
                itemCount: _last.length,
                separatorBuilder: (_, __) => const Divider(height: 1),
                itemBuilder: (_, i) {
                  final e = _last[i];
                  return ListTile(
                    leading: CircleAvatar(child: Text(_typeLabel(e.type))),
                    title: Text(e.note.isEmpty ? "(sin nota)" : e.note),
                    subtitle: Text(e.ts.toString()),
                    trailing: Text(e.amount.toStringAsFixed(2)),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }
}

class _KpiCard extends StatelessWidget {
  final String title;
  final String value;
  const _KpiCard({required this.title, required this.value});

  @override
  Widget build(BuildContext context) {
    return Container(
      width: 170,
      padding: const EdgeInsets.all(14),
      decoration: BoxDecoration(
        borderRadius: BorderRadius.circular(16),
        border: Border.all(color: Colors.black12),
      ),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          Text(title, style: const TextStyle(fontSize: 12, color: Colors.black54)),
          const SizedBox(height: 8),
          Text(value, style: const TextStyle(fontSize: 18, fontWeight: FontWeight.bold)),
        ],
      ),
    );
  }
}
