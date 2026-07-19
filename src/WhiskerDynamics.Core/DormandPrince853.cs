// Adapted from DOP853 by E. Hairer and G. Wanner.
// Copyright (c) 2004, UNIGE. See THIRD-PARTY-NOTICES for source and license details.

namespace WhiskerDynamics.Core;

/// <summary>Adaptive DOP853 RK8(5,3) integrator used by the high-order comparison
/// benchmark. It does not provide DOP853 dense output.</summary>
internal static class DormandPrince853
{
    // Stage abscissae.
    private const double C2 = 0.0526001519587677318785587544488;
    private const double C3 = 0.0789002279381515978178381316732;
    private const double C4 = 0.118350341907227396726757197510;
    private const double C5 = 0.281649658092772603273242802490;
    private const double C6 = 1.0 / 3.0;
    private const double C7 = 0.25;
    private const double C8 = 0.307692307692307692307692307692;
    private const double C9 = 0.651282051282051282051282051282;
    private const double C10 = 0.6;
    private const double C11 = 0.857142857142857142857142857142;

    // 8th-order solution weights.
    private const double B1 = 5.42937341165687622380535766363e-2;
    private const double B6 = 4.45031289275240888144113950566;
    private const double B7 = 1.89151789931450038304281599044;
    private const double B8 = -5.8012039600105847814672114227;
    private const double B9 = 3.1116436695781989440891606237e-1;
    private const double B10 = -1.52160949662516078556178806805e-1;
    private const double B11 = 2.01365400804030348374776537501e-1;
    private const double B12 = 4.47106157277725905176885569043e-2;

    // Embedded 3rd-order error weights (err2 in dop853.f).
    private const double Bhh1 = 0.244094488188976377952755905512;
    private const double Bhh2 = 0.733846688281611857341361741547;
    private const double Bhh3 = 0.0220588235294117647058823529412;

    // Embedded 5th-order error weights.
    private const double Er1 = 0.01312004499419488073250102996;
    private const double Er6 = -1.225156446376204440720569753;
    private const double Er7 = -0.4957589496572501915214079952;
    private const double Er8 = 1.664377182454986536961530415;
    private const double Er9 = -0.3503288487499736816886487290;
    private const double Er10 = 0.3341791187130174790297318841;
    private const double Er11 = 0.08192320648511571246570742613;
    private const double Er12 = -0.02235530786388629525884427845;

    // Runge-Kutta matrix (sparse rows as used by the reference implementation).
    private const double A21 = 5.26001519587677318785587544488e-2;
    private const double A31 = 1.97250569845378994544595329183e-2;
    private const double A32 = 5.91751709536136983633785987549e-2;
    private const double A41 = 2.95875854768068491816892993775e-2;
    private const double A43 = 8.87627564304205475450678981324e-2;
    private const double A51 = 2.41365134159266685502369798665e-1;
    private const double A53 = -8.84549479328286085344864962717e-1;
    private const double A54 = 9.24834003261792003115737966543e-1;
    private const double A61 = 3.7037037037037037037037037037e-2;
    private const double A64 = 1.70828608729473871279604482173e-1;
    private const double A65 = 1.25467687566822425016691814123e-1;
    private const double A71 = 3.7109375e-2;
    private const double A74 = 1.70252211019544039314978060272e-1;
    private const double A75 = 6.02165389804559606850219397283e-2;
    private const double A76 = -1.7578125e-2;
    private const double A81 = 3.70920001185047927108779319836e-2;
    private const double A84 = 1.70383925712239993810214054705e-1;
    private const double A85 = 1.07262030446373284651809199168e-1;
    private const double A86 = -1.53194377486244017527936158236e-2;
    private const double A87 = 8.27378916381402288758473766002e-3;
    private const double A91 = 6.24110958716075717114429577812e-1;
    private const double A94 = -3.36089262944694129406857109825;
    private const double A95 = -8.68219346841726006818189891453e-1;
    private const double A96 = 2.75920996994467083049415600797e1;
    private const double A97 = 2.01540675504778934086186788979e1;
    private const double A98 = -4.34898841810699588477366255144e1;
    private const double A101 = 4.77662536438264365890433908527e-1;
    private const double A104 = -2.48811461997166764192642586468;
    private const double A105 = -5.90290826836842996371446475743e-1;
    private const double A106 = 2.12300514481811942347288949897e1;
    private const double A107 = 1.52792336328824235832596922938e1;
    private const double A108 = -3.32882109689848629194453265587e1;
    private const double A109 = -2.03312017085086261358222928593e-2;
    private const double A111 = -9.3714243008598732571704021658e-1;
    private const double A114 = 5.18637242884406370830023853209;
    private const double A115 = 1.09143734899672957818500254654;
    private const double A116 = -8.14978701074692612513997267357;
    private const double A117 = -1.85200656599969598641566180701e1;
    private const double A118 = 2.27394870993505042818970056734e1;
    private const double A119 = 2.49360555267965238987089396762;
    private const double A1110 = -3.0467644718982195003823669022;
    private const double A121 = 2.27331014751653820792359768449;
    private const double A124 = -1.05344954667372501984066689879e1;
    private const double A125 = -2.00087205822486249909675718444;
    private const double A126 = -1.79589318631187989172765950534e1;
    private const double A127 = 2.79488845294199600508499808837e1;
    private const double A128 = -2.85899827713502369474065508674;
    private const double A129 = -8.87285693353062954433549289258;
    private const double A1210 = 1.23605671757943030647266201528e1;
    private const double A1211 = 6.43392746015763530355970484046e-1;

    /// <summary>Propagates a state and reports the number of RHS evaluations.</summary>
    internal static StateVector Propagate(
        Func<double, StateVector, Vector3d> acceleration,
        StateVector y0, double t0, double t1,
        out long rhsEvaluations,
        IntegratorOptions? options = null,
        Action<double, StateVector>? onAcceptedStep = null)
    {
        if (!double.IsFinite(t0))
            throw new ArgumentOutOfRangeException(nameof(t0), "Initial time must be finite.");
        if (!double.IsFinite(t1))
            throw new ArgumentOutOfRangeException(nameof(t1), "Final time must be finite.");
        if (t1 < t0) throw new ArgumentException("Backward integration is not supported (t1 < t0).");
        if (!y0.IsFinite())
            throw new ArgumentException("Initial state must be finite.", nameof(y0));
        var opt = IntegratorOptions.Validate(options);
        if (t1 == t0) { rhsEvaluations = 0; return y0; }

        double t = t0;
        var y = y0;
        double h = Math.Min(Math.Min(opt.InitialStep, opt.MaxStep), Math.Max(t1 - t0, double.Epsilon));
        long rhs = 0;

        StateVector F(double time, StateVector state)
        {
            rhs++;
            return new StateVector(state.Velocity, acceleration(time, state));
        }

        var k1 = F(t, y);
        int consecutiveRejections = 0;

        while (t < t1)
        {
            h = Math.Min(h, t1 - t);

            var k2 = F(t + C2 * h, y + k1 * (h * A21));
            var k3 = F(t + C3 * h, y + (k1 * A31 + k2 * A32) * h);
            var k4 = F(t + C4 * h, y + (k1 * A41 + k3 * A43) * h);
            var k5 = F(t + C5 * h, y + (k1 * A51 + k3 * A53 + k4 * A54) * h);
            var k6 = F(t + C6 * h, y + (k1 * A61 + k4 * A64 + k5 * A65) * h);
            var k7 = F(t + C7 * h, y + (k1 * A71 + k4 * A74 + k5 * A75 + k6 * A76) * h);
            var k8 = F(t + C8 * h, y + (k1 * A81 + k4 * A84 + k5 * A85 + k6 * A86 + k7 * A87) * h);
            var k9 = F(t + C9 * h, y + (k1 * A91 + k4 * A94 + k5 * A95 + k6 * A96 + k7 * A97 + k8 * A98) * h);
            var k10 = F(t + C10 * h, y + (k1 * A101 + k4 * A104 + k5 * A105 + k6 * A106 + k7 * A107
                                          + k8 * A108 + k9 * A109) * h);
            var k11 = F(t + C11 * h, y + (k1 * A111 + k4 * A114 + k5 * A115 + k6 * A116 + k7 * A117
                                          + k8 * A118 + k9 * A119 + k10 * A1110) * h);
            double tph = t + h;
            var k12 = F(tph, y + (k1 * A121 + k4 * A124 + k5 * A125 + k6 * A126 + k7 * A127
                                  + k8 * A128 + k9 * A129 + k10 * A1210 + k11 * A1211) * h);

            // 8th-order solution and the two embedded error estimators.
            var kb = k1 * B1 + k6 * B6 + k7 * B7 + k8 * B8 + k9 * B9 + k10 * B10 + k11 * B11 + k12 * B12;
            var yNew = y + kb * h;
            var er5 = k1 * Er1 + k6 * Er6 + k7 * Er7 + k8 * Er8 + k9 * Er9 + k10 * Er10 + k11 * Er11 + k12 * Er12;
            var er3 = kb + k1 * -Bhh1 + k9 * -Bhh2 + k12 * -Bhh3;

            double err = ErrorNorm(y, yNew, er5, er3, h, opt);

            if (double.IsNaN(err) || double.IsInfinity(err))
            {
                if (++consecutiveRejections > 60)
                    throw new IntegrationFailureException($"Integration diverged (NaN) at t={t}.");
                h *= 0.2;
                // Match DP54's underflow behavior so comparisons terminate identically.
                if (h < DormandPrince54.MinStep(t))
                    throw new IntegrationFailureException($"Step size underflow at t={t} — dynamics too stiff or divergent.");
                continue;
            }

            // DOP853 controller (beta = 0): grow up to 6x, shrink down to 1/3, safety 0.9.
            double fac11 = Math.Pow(err, 1.0 / 8.0);
            double fac = Math.Max(1.0 / 6.0, Math.Min(3.0, fac11 / 0.9));

            if (err <= 1.0)
            {
                consecutiveRejections = 0;
                t = tph;
                y = yNew;
                k1 = F(t, y); // derivative at the accepted point seeds the next step
                onAcceptedStep?.Invoke(t, y);
                h /= fac;
            }
            else
            {
                if (++consecutiveRejections > 60)
                    throw new IntegrationFailureException($"Integration failed: 60 consecutive step rejections at t={t}.");
                h /= Math.Min(3.0, fac11 / 0.9); // never grow a rejected step
            }

            h = Math.Min(h, opt.MaxStep);
            if (h < DormandPrince54.MinStep(t) && t < t1)
                throw new IntegrationFailureException($"Step size underflow at t={t} — dynamics too stiff or divergent.");
        }

        rhsEvaluations = rhs;
        return y;
    }

    /// <summary>DOP853's combined 5th/3rd-order error norm, adapted to this codebase's
    /// per-kind absolute tolerances: err = |h|·err5·sqrt(1/(n·(err5 + 0.01·err3))) with
    /// err5/err3 the tolerance-scaled sums of squares over the 6 state components.</summary>
    private static double ErrorNorm(StateVector y, StateVector yNew, StateVector er5, StateVector er3,
        double h, IntegratorOptions opt)
    {
        Span<double> now = [y.Position.X, y.Position.Y, y.Position.Z, y.Velocity.X, y.Velocity.Y, y.Velocity.Z];
        Span<double> next = [yNew.Position.X, yNew.Position.Y, yNew.Position.Z, yNew.Velocity.X, yNew.Velocity.Y, yNew.Velocity.Z];
        Span<double> e5 = [er5.Position.X, er5.Position.Y, er5.Position.Z, er5.Velocity.X, er5.Velocity.Y, er5.Velocity.Z];
        Span<double> e3 = [er3.Position.X, er3.Position.Y, er3.Position.Z, er3.Velocity.X, er3.Velocity.Y, er3.Velocity.Z];

        double err5 = 0, err3 = 0;
        for (int i = 0; i < 6; i++)
        {
            double absTol = i < 3 ? opt.AbsTolPos : opt.AbsTolVel;
            double sk = absTol + opt.RelTol * Math.Max(Math.Abs(now[i]), Math.Abs(next[i]));
            double a = e5[i] / sk;
            double b = e3[i] / sk;
            err5 += a * a;
            err3 += b * b;
        }
        double deno = err5 + 0.01 * err3;
        if (deno <= 0) deno = 1.0;
        return Math.Abs(h) * err5 * Math.Sqrt(1.0 / (6 * deno));
    }
}
