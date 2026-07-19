using WhiskerDynamics.GameTestDriver.Runtime;

namespace WhiskerDynamics.GameTestHost.Tests;

public class GameTestLunarTransferSolveJobTests
{
    [Fact]
    public void Departure_search_keeps_the_requested_flight_duration()
    {
        const double departureStart = 1_000;
        const double searchDuration = 7_200;
        const double flightDuration = 3 * 86_400;
        const int sampleCount = 25;

        for (int i = 0; i < sampleCount; i++)
        {
            var times = GameTestLunarTransferSolveJob.CandidateTimes(
                departureStart, searchDuration, flightDuration, i, sampleCount);
            Assert.Equal(flightDuration,
                times.ArrivalTime - times.DepartureTime, precision: 9);
        }

        var last = GameTestLunarTransferSolveJob.CandidateTimes(
            departureStart, searchDuration, flightDuration,
            sampleCount - 1, sampleCount);
        Assert.Equal(departureStart + searchDuration,
            last.DepartureTime, precision: 9);
        Assert.Equal(departureStart + searchDuration + flightDuration,
            last.ArrivalTime, precision: 9);
    }
}
