import "package:hive_flutter/hive_flutter.dart";

class HiveService {
  static Future<void> init() async {
    await Hive.initFlutter();
  }
}
