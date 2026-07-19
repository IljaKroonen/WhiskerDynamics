namespace WhiskerDynamics.Core;

/// <summary>An adaptive integration could not advance because the dynamics became
/// non-finite or required a step smaller than the representable/supported floor.</summary>
public sealed class IntegrationFailureException(string message) : InvalidOperationException(message);
