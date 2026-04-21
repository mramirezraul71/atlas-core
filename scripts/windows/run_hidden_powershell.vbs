Option Explicit

If WScript.Arguments.Count < 1 Then
  WScript.Quit 1
End If

Dim scriptPath, extraArgs, i
scriptPath = WScript.Arguments(0)
extraArgs = ""

If WScript.Arguments.Count > 1 Then
  For i = 1 To WScript.Arguments.Count - 1
    extraArgs = extraArgs & " " & WScript.Arguments(i)
  Next
End If

Dim shell, cmd
Set shell = CreateObject("WScript.Shell")
cmd = "powershell.exe -NoProfile -NonInteractive -ExecutionPolicy Bypass -File """ & scriptPath & """" & extraArgs

' WindowStyle 0 = hidden, WaitOnReturn False = background.
shell.Run cmd, 0, False
