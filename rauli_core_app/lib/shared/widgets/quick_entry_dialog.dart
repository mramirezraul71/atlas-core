import "package:flutter/material.dart";

class QuickEntryDialog extends StatefulWidget {
  final String title;
  final String amountLabel;
  final String noteLabel;

  const QuickEntryDialog({
    super.key,
    required this.title,
    required this.amountLabel,
    required this.noteLabel,
  });

  @override
  State<QuickEntryDialog> createState() => _QuickEntryDialogState();
}

class _QuickEntryDialogState extends State<QuickEntryDialog> {
  final _amount = TextEditingController();
  final _note = TextEditingController();

  @override
  void dispose() {
    _amount.dispose();
    _note.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return AlertDialog(
      title: Text(widget.title),
      content: Column(
        mainAxisSize: MainAxisSize.min,
        children: [
          TextField(
            controller: _amount,
            keyboardType: const TextInputType.numberWithOptions(decimal: true),
            decoration: InputDecoration(labelText: widget.amountLabel),
          ),
          const SizedBox(height: 12),
          TextField(
            controller: _note,
            decoration: InputDecoration(labelText: widget.noteLabel),
          ),
        ],
      ),
      actions: [
        TextButton(
          onPressed: () => Navigator.pop(context),
          child: const Text("Cancelar"),
        ),
        FilledButton(
          onPressed: () {
            final a = double.tryParse(_amount.text.trim()) ?? 0.0;
            final n = _note.text.trim();
            Navigator.pop(context, {"amount": a, "note": n});
          },
          child: const Text("Guardar"),
        )
      ],
    );
  }
}
