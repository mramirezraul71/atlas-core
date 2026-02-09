import "package:flutter/material.dart";
import "../../shared/services/hive_service.dart";
import "../router/app_router.dart";
import "../theme/app_theme.dart";

class AppBootstrap {
  static Future<void> run() async {
    WidgetsFlutterBinding.ensureInitialized();
    await HiveService.init();
    runApp(const RauliApp());
  }
}

class RauliApp extends StatelessWidget {
  const RauliApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: "RAULI",
      debugShowCheckedModeBanner: false,
      theme: AppTheme.light,
      onGenerateRoute: AppRouter.generate,
    );
  }
}
