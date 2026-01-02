# Security Policy

## Supported Versions

We release patches for security vulnerabilities. The following versions are currently supported:

| Version | Supported          |
| ------- | ------------------ |
| main    | :white_check_mark: |
| < 1.0   | :x:                |

---

## Reporting a Vulnerability

**Please do NOT report security vulnerabilities through public GitHub issues.**

Instead, please report security vulnerabilities via:

### Option 1: GitHub Security Advisory (Preferred)

1. Navigate to the repository's **Security** tab
2. Click **"Report a vulnerability"**
3. Fill out the security advisory form

### Option 2: Email

Send an email to: **security@yourproject.com** *(replace with actual contact)*

Include as much information as possible:
- Type of vulnerability
- Full paths of source files related to the vulnerability
- Location of affected source code (tag/branch/commit)
- Step-by-step instructions to reproduce
- Proof-of-concept or exploit code (if possible)
- Impact of the vulnerability

---

## What to Expect

### Response Timeline

- **Initial Response**: Within 48 hours
- **Status Update**: Within 7 days
- **Fix Timeline**: Varies by severity (see below)

### Severity Levels

| Severity | Response Time | Fix Target |
|----------|--------------|------------|
| **Critical** | 24 hours | 7 days |
| **High** | 48 hours | 14 days |
| **Medium** | 7 days | 30 days |
| **Low** | 14 days | 60 days |

---

## Security Considerations for HoverBot

### Physical Safety

HoverBot is a **physical robot** that can cause harm if compromised. Security vulnerabilities can lead to:

⚠️ **Unintended motion** - Could cause injury or property damage
⚠️ **Loss of control** - Remote attackers controlling the robot
⚠️ **Data exposure** - Maps, camera feeds, sensor data

### Known Security Considerations

1. **Serial Communication**
   - No authentication on UART connection
   - Physical access to pins = full control
   - **Mitigation**: Physical security of the robot

2. **ROS 2 Network**
   - ROS 2 DDS multicast can be discovered on network
   - No encryption by default
   - **Mitigation**: Use isolated network, enable DDS security

3. **SSH Access**
   - Default credentials on Raspberry Pi/Jetson
   - **Mitigation**: Change default passwords immediately

4. **Sensor Data**
   - LiDAR and cameras collect environmental data
   - Could reveal sensitive location information
   - **Mitigation**: Secure storage, encrypted transmission

---

## Security Best Practices

### For Users

**Before Deployment:**
- [ ] Change all default passwords
- [ ] Update all system packages
- [ ] Enable firewall on Raspberry Pi/Jetson
- [ ] Use isolated network for robot
- [ ] Enable ROS 2 DDS security (if needed)

**During Operation:**
- [ ] Monitor for unexpected behavior
- [ ] Keep software up to date
- [ ] Review logs regularly
- [ ] Implement emergency stop mechanism

### For Developers

**Code Security:**
- [ ] Validate all serial input
- [ ] Sanitize sensor data before publishing
- [ ] Implement velocity limits
- [ ] Add timeout mechanisms
- [ ] Use secure coding practices

**Testing:**
- [ ] Test failure modes
- [ ] Verify emergency stop
- [ ] Check input validation
- [ ] Test timeout behavior

---

## Vulnerability Disclosure Process

1. **Report received** - We acknowledge receipt
2. **Triage** - We assess severity and impact
3. **Investigation** - We reproduce and analyze
4. **Fix Development** - We develop and test a patch
5. **Coordinated Disclosure** - We prepare advisory and release
6. **Public Disclosure** - We publish security advisory and release patch

### Coordinated Disclosure Timeline

- **Day 0**: Vulnerability reported
- **Day 1-2**: Initial triage and acknowledgment
- **Day 3-7**: Investigation and reproduction
- **Day 7-30**: Fix development (varies by severity)
- **Day 30+**: Coordinated public disclosure

We aim to disclose vulnerabilities within **90 days** of initial report.

---

## Security Updates

Security updates are released as:
- **Patch releases** for supported versions
- **Security advisories** on GitHub
- **Announcements** in README and discussions

Subscribe to repository releases to be notified of security updates.

---

## Hall of Fame

We recognize security researchers who responsibly disclose vulnerabilities:

<!-- Contributors who report security issues will be listed here -->

*No reports yet*

---

## Scope

### In Scope

- ROS 2 nodes and drivers
- Serial communication protocol
- Launch files and configurations
- Documentation that could mislead users into insecure practices

### Out of Scope

- Third-party dependencies (report to upstream)
- Physical access attacks (assumed secure physical environment)
- Denial of service via physical interference
- Issues in ROS 2 core or Linux kernel (report upstream)

---

## Legal

We will not pursue legal action against security researchers who:
- Make good faith efforts to comply with this policy
- Do not access data beyond what is necessary to demonstrate the vulnerability
- Do not harm users or degrade user experience
- Report vulnerabilities promptly
- Keep vulnerabilities confidential until public disclosure

---

## Contact

- **Security Email**: security@yourproject.com *(replace with actual contact)*
- **GitHub Security**: Use "Report a vulnerability" feature
- **PGP Key**: Available upon request

---

**Thank you for helping keep HoverBot and its users safe!** 🔒
