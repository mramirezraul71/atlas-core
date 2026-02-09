import "dart:math";
import "package:hive/hive.dart";
import "../models/entry_model.dart";

class RauliRepo {
  static const String salesBoxName = "sales_box";
  static const String prodBoxName = "production_box";

  static Future<Box> _open(String name) async {
    if (Hive.isBoxOpen(name)) return Hive.box(name);
    return await Hive.openBox(name);
  }

  static String _id() {
    final r = Random();
    final n = r.nextInt(1 << 30);
    return "${DateTime.now().millisecondsSinceEpoch}_$n";
  }

  static Future<void> addSale({required double amount, required String note}) async {
    final box = await _open(salesBoxName);
    final e = EntryModel(id: _id(), type: "sale", note: note, amount: amount, ts: DateTime.now());
    await box.put(e.id, e.toMap());
  }

  static Future<void> addProduction({required double amount, required String note}) async {
    final box = await _open(prodBoxName);
    final e = EntryModel(id: _id(), type: "production", note: note, amount: amount, ts: DateTime.now());
    await box.put(e.id, e.toMap());
  }

  static Future<List<EntryModel>> lastEntries({int limit = 20}) async {
    final sales = await _open(salesBoxName);
    final prod = await _open(prodBoxName);

    final List<EntryModel> all = [];

    for (final k in sales.keys) {
      final v = sales.get(k);
      if (v is Map) all.add(EntryModel.fromMap(v));
    }
    for (final k in prod.keys) {
      final v = prod.get(k);
      if (v is Map) all.add(EntryModel.fromMap(v));
    }

    all.sort((a, b) => b.ts.compareTo(a.ts));
    return all.take(limit).toList();
  }

  static bool _isSameDay(DateTime a, DateTime b) =>
      a.year == b.year && a.month == b.month && a.day == b.day;

  static Future<double> totalSalesToday() async {
    final sales = await _open(salesBoxName);
    double sum = 0.0;
    for (final k in sales.keys) {
      final v = sales.get(k);
      if (v is Map) {
        final e = EntryModel.fromMap(v);
        if (_isSameDay(e.ts, DateTime.now())) sum += e.amount;
      }
    }
    return sum;
  }

  static Future<double> totalProductionToday() async {
    final prod = await _open(prodBoxName);
    double sum = 0.0;
    for (final k in prod.keys) {
      final v = prod.get(k);
      if (v is Map) {
        final e = EntryModel.fromMap(v);
        if (_isSameDay(e.ts, DateTime.now())) sum += e.amount;
      }
    }
    return sum;
  }
}
