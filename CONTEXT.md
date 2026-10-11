# Project glossary

## Verification

- **Build configuration:** one concrete combination of build type, scalar precision,
  sanitizer instrumentation, Qhull mode, and internal-state policy.
- **Verification suite:** a named group of checks that runs in one build
  configuration and has an explicit expected result.
- **Test set:** the exact CTest entries selected by a verification suite.
- **Expected result:** the observable result required for a verification
  suite to pass.
- **Run result:** the selected tests, commands, outputs, and pass/fail status
  from one suite run.
- **Untested behavior:** behavior for which no existing test defines an
  expected result.
