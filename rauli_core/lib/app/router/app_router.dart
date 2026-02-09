import "package:flutter/material.dart";
import "../../features/dashboard/dashboard_page.dart";

class AppRouter {
  static Route<dynamic> generate(RouteSettings settings) {
    switch (settings.name) {
      case "/":
      default:
        return MaterialPageRoute(builder: (_) => const DashboardPage());
    }
  }
}
