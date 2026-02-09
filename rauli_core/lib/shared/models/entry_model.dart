class EntryModel {
  final String id;
  final String type; // "sale" | "production"
  final String note;
  final double amount;
  final DateTime ts;

  EntryModel({
    required this.id,
    required this.type,
    required this.note,
    required this.amount,
    required this.ts,
  });

  Map<String, dynamic> toMap() => {
        "id": id,
        "type": type,
        "note": note,
        "amount": amount,
        "ts": ts.toIso8601String(),
      };

  static EntryModel fromMap(Map<dynamic, dynamic> m) => EntryModel(
        id: (m["id"] ?? "").toString(),
        type: (m["type"] ?? "").toString(),
        note: (m["note"] ?? "").toString(),
        amount: (m["amount"] is num) ? (m["amount"] as num).toDouble() : 0.0,
        ts: DateTime.tryParse((m["ts"] ?? "").toString()) ?? DateTime.now(),
      );
}
